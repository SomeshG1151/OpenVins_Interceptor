#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <deque>
#include <algorithm>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

struct AxisFilter {
  std::deque<double> window;
  int median_window_size = 3;
  bool use_median = true;
  int order = 2;
  double alpha = 0.3;
  double stage1 = 0.0;
  double stage2 = 0.0;
  bool initialized = false;

  void configure(double a, int ord, bool median_on, int med_win) {
    alpha = a;
    order = ord;
    use_median = median_on;
    median_window_size = std::max(1, med_win);
  }

  double update(double raw) {
    double val = raw;
    if (use_median && median_window_size > 1) {
      window.push_back(raw);
      if (window.size() > static_cast<size_t>(median_window_size)) {
        window.pop_front();
      }
      std::vector<double> sorted(window.begin(), window.end());
      std::sort(sorted.begin(), sorted.end());
      val = sorted[sorted.size() / 2];
    }

    if (!initialized) {
      stage1 = val;
      stage2 = val;
      initialized = true;
      return val;
    }

    // Stage 1 low-pass (1st order EMA)
    stage1 = alpha * val + (1.0 - alpha) * stage1;

    // Stage 2 low-pass (cascaded 2nd order for steep high-frequency roll-off)
    if (order >= 2) {
      stage2 = alpha * stage1 + (1.0 - alpha) * stage2;
      return stage2;
    }
    return stage1;
  }

  void reset() {
    window.clear();
    initialized = false;
  }
};

class ESP32IMUDriver : public rclcpp::Node
{
public:
  ESP32IMUDriver() : Node("esp32_imu_driver"), fd_(-1)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<double>("accel_scale", 1.0);
    this->declare_parameter<double>("gyro_scale", 1.0);
    this->declare_parameter<bool>("use_filter", false);
    this->declare_parameter<double>("filter_alpha", 0.80);
    this->declare_parameter<double>("filter_alpha_accel", 0.80);
    this->declare_parameter<double>("filter_alpha_gyro", 0.80);
    this->declare_parameter<int>("filter_order", 1);
    this->declare_parameter<bool>("use_median_filter", false);
    this->declare_parameter<int>("median_window", 3);

    std::string port = this->get_parameter("port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    accel_scale_ = this->get_parameter("accel_scale").as_double();
    gyro_scale_ = this->get_parameter("gyro_scale").as_double();
    use_filter_ = this->get_parameter("use_filter").as_bool();
    filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    filter_alpha_accel_ = this->get_parameter("filter_alpha_accel").as_double();
    filter_alpha_gyro_ = this->get_parameter("filter_alpha_gyro").as_double();
    filter_order_ = this->get_parameter("filter_order").as_int();
    use_median_filter_ = this->get_parameter("use_median_filter").as_bool();
    median_window_ = this->get_parameter("median_window").as_int();

    // If generic filter_alpha was explicitly tuned away from default 0.30 but specific ones weren't, align them
    if (this->has_parameter("filter_alpha") && filter_alpha_ != 0.30) {
      if (!this->has_parameter("filter_alpha_accel")) filter_alpha_accel_ = filter_alpha_;
      if (!this->has_parameter("filter_alpha_gyro")) filter_alpha_gyro_ = filter_alpha_;
    }

    if (use_filter_) {
      RCLCPP_INFO(this->get_logger(),
                  "IMU filter ENABLED: Order=%d, MedianFilter=%s (window=%d), Alpha(accel=%.2f, gyro=%.2f)",
                  filter_order_, use_median_filter_ ? "ON" : "OFF", median_window_,
                  filter_alpha_accel_, filter_alpha_gyro_);
      initFilters();
    } else {
      RCLCPP_WARN(this->get_logger(), "IMU filter DISABLED (raw data stream)");
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    // Auto-detect port if configured port doesn't exist (e.g. Linux re-enumerated ttyACM0 to ttyACM1)
    if (::access(port.c_str(), F_OK) != 0) {
      std::vector<std::string> candidates = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyUSB0", "/dev/ttyUSB1"};
      for (const auto &cand : candidates) {
        if (::access(cand.c_str(), F_OK) == 0) {
          RCLCPP_WARN(this->get_logger(), "Port %s not found. Auto-detected active ESP32 at %s!", port.c_str(), cand.c_str());
          port = cand;
          break;
        }
      }
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port.c_str(), strerror(errno));
      return;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd_, TCSANOW, &tty);

    // Pulse DTR/RTS to release ESP32 auto-reset
    int flags = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd_, TIOCMBIC, &flags);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    flags = TIOCM_DTR;
    ioctl(fd_, TIOCMBIS, &flags);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    tcflush(fd_, TCIFLUSH);

    RCLCPP_INFO(this->get_logger(), "Serial port %s opened and ESP32 initialized", port.c_str());

    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&ESP32IMUDriver::readSerial, this));
  }

  ~ESP32IMUDriver()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  void readSerial()
  {
    if (fd_ < 0) return;

    char buf[512];
    int n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      buffer_.append(buf, n);
      parseBuffer();
    }
  }

  void parseBuffer()
  {
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);
      if (line.empty()) continue;

      std::replace(line.begin(), line.end(), ',', ' ');
      std::stringstream ss(line);
      double val;
      std::vector<double> values;
      while (ss >> val) {
        values.push_back(val);
      }

      if (values.size() >= 6) {
        publishIMU(values);
      }
    }
  }

  void initFilters()
  {
    filt_ax_.configure(filter_alpha_accel_, filter_order_, use_median_filter_, median_window_);
    filt_ay_.configure(filter_alpha_accel_, filter_order_, use_median_filter_, median_window_);
    filt_az_.configure(filter_alpha_accel_, filter_order_, use_median_filter_, median_window_);

    filt_gx_.configure(filter_alpha_gyro_, filter_order_, use_median_filter_, median_window_);
    filt_gy_.configure(filter_alpha_gyro_, filter_order_, use_median_filter_, median_window_);
    filt_gz_.configure(filter_alpha_gyro_, filter_order_, use_median_filter_, median_window_);
  }

  void publishIMU(const std::vector<double>& values)
  {
    auto msg = sensor_msgs::msg::Imu();
    
    // Ensure strictly monotonic timestamps to avoid zero-dt drops in OpenVINS
    rclcpp::Time current_time = this->now();
    if (last_stamp_.nanoseconds() > 0 && current_time <= last_stamp_) {
      current_time = last_stamp_ + rclcpp::Duration::from_nanoseconds(10000000); // 10ms step for 100 Hz
    }
    last_stamp_ = current_time;
    msg.header.stamp = current_time;
    msg.header.frame_id = frame_id_;

    double raw_ax = values[0] * accel_scale_;
    double raw_ay = values[1] * accel_scale_;
    double raw_az = values[2] * accel_scale_;

    double raw_gx = values[3] * gyro_scale_;
    double raw_gy = values[4] * gyro_scale_;
    double raw_gz = values[5] * gyro_scale_;

    if (use_filter_) {
      msg.linear_acceleration.x = filt_ax_.update(raw_ax);
      msg.linear_acceleration.y = filt_ay_.update(raw_ay);
      msg.linear_acceleration.z = filt_az_.update(raw_az);

      msg.angular_velocity.x = filt_gx_.update(raw_gx);
      msg.angular_velocity.y = filt_gy_.update(raw_gy);
      msg.angular_velocity.z = filt_gz_.update(raw_gz);
    } else {
      msg.linear_acceleration.x = raw_ax;
      msg.linear_acceleration.y = raw_ay;
      msg.linear_acceleration.z = raw_az;

      msg.angular_velocity.x = raw_gx;
      msg.angular_velocity.y = raw_gy;
      msg.angular_velocity.z = raw_gz;
    }

    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.orientation_covariance[0] = -1;

    // Filtered data has lower variance
    msg.linear_acceleration_covariance[0] = 0.005;
    msg.linear_acceleration_covariance[4] = 0.005;
    msg.linear_acceleration_covariance[8] = 0.005;
    msg.angular_velocity_covariance[0] = 0.0005;
    msg.angular_velocity_covariance[4] = 0.0005;
    msg.angular_velocity_covariance[8] = 0.0005;

    imu_pub_->publish(msg);
  }

  int fd_;
  std::string buffer_;
  std::string frame_id_;
  double accel_scale_;
  double gyro_scale_;
  bool use_filter_;
  double filter_alpha_;
  double filter_alpha_accel_;
  double filter_alpha_gyro_;
  int filter_order_;
  bool use_median_filter_;
  int median_window_;

  AxisFilter filt_ax_, filt_ay_, filt_az_;
  AxisFilter filt_gx_, filt_gy_, filt_gz_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESP32IMUDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}