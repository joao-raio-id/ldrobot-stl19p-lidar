
#include "ros2_api.h"
#include "ldlidar_driver/ldlidar_driver_linux.h"
#include "ldlidar_driver/ldlidar_driver.h"

class LDLidarNode : public rclcpp::Node {

public:
  LDLidarNode() : Node("ldlidar_node") {
    RCLCPP_INFO(get_logger(), "LDLiDAR ROS2 node start.");

    declareParameters();
    getParameters();

    ldlidar_drv = ldlidar::LDLidarDriverLinuxInterface::Create();

    printParameters();

    lidar_pub_laserscan = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic_name, 10);
    
    lidar_pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud>(point_cloud_2d_topic_name, 10);

    if (!initLidarDriver() || !startLidarDriver()) {
      throw std::runtime_error("LDLidar driver init/start failed");
    }

    scan_thread = std::thread(&LDLidarNode::publishLidarData, this);

  }
  ~LDLidarNode(){
  ldlidar_drv->Stop();
  ldlidar_drv->Disconnect();

  if (scan_thread.joinable()) {
      scan_thread.join();
  }

  ldlidar::LDLidarDriverLinuxInterface::Destory(ldlidar_drv);

  RCLCPP_INFO(this->get_logger(), "ldlidar published is end");
  rclcpp::shutdown();
  };

private:

  void declareParameters(){
    RCLCPP_INFO(get_logger(), "Declaring parameters.");
  this->declare_parameter<std::string>("product_name", "LDLiDAR_STL19P");
  this->declare_parameter<std::string>("laser_scan_topic_name", "scan");
  this->declare_parameter<std::string>("point_cloud_2d_topic_name", "point_cloud_2d");
  this->declare_parameter<std::string>("frame_id", "base_laser");
  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<double>("lidar_scan_freq", 10.0);
  this->declare_parameter<int>("serial_baudrate", 230400);
  this->declare_parameter<bool>("laser_scan_dir", true);
  this->declare_parameter<bool>("enable_angle_crop_func", false);
  this->declare_parameter<double>("angle_crop_min", 135.0);
  this->declare_parameter<double>("angle_crop_max", 225.0);
  this->declare_parameter<double>("range_min", 0.1);
  this->declare_parameter<double>("range_max", 10.0);
  }

  void getParameters(){
    RCLCPP_INFO(get_logger(), "Getting parameters.");
  this->get_parameter("product_name", product_name);
  this->get_parameter("laser_scan_topic_name", laser_scan_topic_name);
  this->get_parameter("point_cloud_2d_topic_name", point_cloud_2d_topic_name);
  this->get_parameter("frame_id", setting.frame_id);
  this->get_parameter("port_name", port_name);
  this->get_parameter("serial_baudrate", serial_baudrate);
  this->get_parameter("lidar_scan_freq", lidar_scan_freq);
  this->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  this->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  this->get_parameter("angle_crop_min", setting.angle_crop_min);
  this->get_parameter("angle_crop_max", setting.angle_crop_max);
  this->get_parameter("range_min", setting.range_min);
  this->get_parameter("range_max", setting.range_max);
  }

  void printParameters(){
  RCLCPP_INFO(get_logger(), "Printing parameters.");
  RCLCPP_INFO(get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidar_drv->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(get_logger(), "ROS2 param input:");
  RCLCPP_INFO(get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(get_logger(), "<laser_scan_topic_name>: %s", laser_scan_topic_name.c_str());
  RCLCPP_INFO(get_logger(), "<point_cloud_2d_topic_name>: %s", point_cloud_2d_topic_name.c_str());
  RCLCPP_INFO(get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(get_logger(), "<port_name>: %s ", port_name.c_str());
  RCLCPP_INFO(get_logger(), "<serial_baudrate>: %d ", serial_baudrate);
  RCLCPP_INFO(get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);
  RCLCPP_INFO(get_logger(), "<range_min>: %f", setting.range_min);
  RCLCPP_INFO(get_logger(), "<range_max>: %f", setting.range_max);
  RCLCPP_INFO(get_logger(), "<lidar_scan_freq>: %f", lidar_scan_freq);
  }

  bool initLidarDriver(){
    RCLCPP_INFO(get_logger(), "LDLiDAR Init.");
    if (port_name.empty()) {
      RCLCPP_ERROR(get_logger(), "fail, port_name is empty!");
      return false;
    }

    ldlidar_drv->RegisterGetTimestampFunctional(std::bind(&LDLidarNode::GetTimestamp, this));

    ldlidar_drv->EnablePointCloudDataFilter(true);
    
    if(!strcmp(product_name.c_str(),"LDLiDAR_LD14")) {
      lidartypename = ldlidar::LDType::LD_14;
    } else if (!strcmp(product_name.c_str(), "LDLiDAR_LD14P")) {
      lidartypename = ldlidar::LDType::LD_14P;
    } else if (!strcmp(product_name.c_str(),"LDLiDAR_LD06")) {
      lidartypename = ldlidar::LDType::LD_06;
    } else if (!strcmp(product_name.c_str(),"LDLiDAR_LD19")) {
      lidartypename = ldlidar::LDType::LD_19;
    } else if (!strcmp(product_name.c_str(),"LDLiDAR_STL19P")) {
      lidartypename = ldlidar::LDType::LD_19;
    } else {
      RCLCPP_ERROR(get_logger(),"Error, input param <product_name> is fail!!");
      return false;
    }

    if (ldlidar_drv->Connect(lidartypename, port_name, serial_baudrate)) {
      RCLCPP_INFO(get_logger(), "ldlidar serial connect is success");
    } else {
      RCLCPP_ERROR(get_logger(), "ldlidar serial connect is fail");
      return false;
    }

    if (ldlidar_drv->WaitLidarComm(3500)) {
      RCLCPP_INFO(get_logger(), "ldlidar communication is normal.");
    } else {
      RCLCPP_ERROR(get_logger(), "ldlidar communication is abnormal.");
      return false;
    }
    return true;
  }

  bool startLidarDriver(){
    RCLCPP_INFO(get_logger(), "Starting LDLiDAR driver.");
    if (ldlidar_drv->Start()) {
      RCLCPP_INFO(get_logger(), "ldlidar driver start is success.");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "ldlidar driver start is fail.");
      return false;
  }
  }

  bool stopLidarDriver(){
    if (!ldlidar_drv->Stop()) {
      RCLCPP_INFO(get_logger(), "ldlidar driver stop is success.");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "ldlidar driver stop is fail.");
      return false;
    }
  }

  bool disconnectLidarDriver(){
    if (!ldlidar_drv->Disconnect()) {
      RCLCPP_INFO(get_logger(), "ldlidar driver disconnect is success.");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "ldlidar driver disconnect is fail.");
      return false;
    }
  }

  void publishLidarData(){
  RCLCPP_INFO(get_logger(), "start normal, pub lidar data");
  rclcpp::WallRate rate(lidar_scan_freq);

  while (rclcpp::ok() && 
    ldlidar::LDLidarDriver::Ok()) {
  
    switch (ldlidar_drv->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_scan_freq = 0;
        ldlidar_drv->GetLidarScanFreq(lidar_scan_freq);
        PublishLaserscan();
        PublishSensorPointCloud();
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        RCLCPP_ERROR(get_logger(), "ldlidar point cloud data publish time out, please check your lidar device.");
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }
    rate.sleep();
  }
  }

  uint64_t GetTimestamp(void) {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
      std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

  void PublishLaserscan() {
    float angle_min, angle_max, range_min, range_max, angle_increment;
    double scan_time;
    rclcpp::Time start_scan_time;
    static rclcpp::Time end_scan_time;
    static bool first_scan = true;

    start_scan_time = this->now();
    scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

    if (first_scan) {
      first_scan = false;
      end_scan_time = start_scan_time;
      return;
    }
    // Adjust the parameters according to the demand
    angle_min = 0;
    angle_max = (2 * M_PI);
    range_min = static_cast<float>(setting.range_min);
    range_max = static_cast<float>(setting.range_max);
    int beam_size = static_cast<int>(laser_scan_points.size());
    if (beam_size <= 1) {
      end_scan_time = start_scan_time;
      RCLCPP_ERROR(get_logger(), "beam_size <= 1");
      return;
    }
    angle_increment = (angle_max - angle_min) / (float)(beam_size -1);
    // Calculate the number of scanning points
    if (lidar_scan_freq > 0) {
      sensor_msgs::msg::LaserScan output;
      output.header.stamp = start_scan_time;
      output.header.frame_id = setting.frame_id;
      output.angle_min = angle_min;
      output.angle_max = angle_max;
      output.range_min = range_min;
      output.range_max = range_max;
      output.angle_increment = angle_increment;
      if (beam_size <= 1) {
        output.time_increment = 0;
      } else {
        output.time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
      }
      output.scan_time = scan_time;
      // First fill all the data with Nan
      output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
      output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
      for (auto point : laser_scan_points) {
        float range = point.distance / 1000.f;  // distance unit transform to meters
        float intensity = point.intensity;      // laser receive intensity 
        float dir_angle = point.angle;

        if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
          range = std::numeric_limits<float>::quiet_NaN(); 
          intensity = std::numeric_limits<float>::quiet_NaN();
        }

        if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
          if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
            range = std::numeric_limits<float>::quiet_NaN();
            intensity = std::numeric_limits<float>::quiet_NaN();
          }
        }

        float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
        int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
        if (index < beam_size) {
          if (index < 0) {
            RCLCPP_ERROR(get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
              index, beam_size, angle, angle_min, angle_increment);
          }

          if (setting.laser_scan_dir) {
            int index_anticlockwise = beam_size - index - 1;
            // If the current content is Nan, it is assigned directly
            if (std::isnan(output.ranges[index_anticlockwise])) {
              output.ranges[index_anticlockwise] = range;
            } else { // Otherwise, only when the distance is less than the current
                      //   value, it can be re assigned
              if (range < output.ranges[index_anticlockwise]) {
                  output.ranges[index_anticlockwise] = range;
              }
            }
            output.intensities[index_anticlockwise] = intensity;
          } else {
            // If the current content is Nan, it is assigned directly
            if (std::isnan(output.ranges[index])) {
              output.ranges[index] = range;
            } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
              if (range < output.ranges[index]) {
                output.ranges[index] = range;
              }
            }
            output.intensities[index] = intensity;
          }
        }
      }
      lidar_pub_laserscan->publish(output);
      end_scan_time = start_scan_time;
    } 
  }

  void PublishSensorPointCloud() {
    
    rclcpp::Time start_scan_time;
    double scan_time;
    float time_increment;
    static rclcpp::Time end_scan_time;
    static bool first_scan = true;

    ldlidar::Points2D dst = laser_scan_points;

    start_scan_time = this->now();
    scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

    if (first_scan) {
      first_scan = false;
      end_scan_time = start_scan_time;
      return;
    }

    if (setting.laser_scan_dir) {
      for (auto&point : dst) {
        point.angle = 360.f - point.angle;
        if (point.angle < 0) {
          point.angle += 360.f;
        }
      }
    } 

    int frame_points_num = static_cast<int>(dst.size());

    sensor_msgs::msg::PointCloud output;

    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;

    sensor_msgs::msg::ChannelFloat32 defaultchannelval[3];

    defaultchannelval[0].name = std::string("intensity");
    defaultchannelval[0].values.assign(frame_points_num, std::numeric_limits<float>::quiet_NaN());
    // output.channels.assign(1, defaultchannelval);
    output.channels.push_back(defaultchannelval[0]);

    if (frame_points_num <= 1) {
      time_increment = 0;
    } else {
      time_increment = static_cast<float>(scan_time / (double)(frame_points_num - 1));
    }
    defaultchannelval[1].name = std::string("timeincrement");
    defaultchannelval[1].values.assign(1, time_increment);
    output.channels.push_back(defaultchannelval[1]);
    
    defaultchannelval[2].name = std::string("scantime");
    defaultchannelval[2].values.assign(1, scan_time);
    output.channels.push_back(defaultchannelval[2]);

    geometry_msgs::msg::Point32 points_xyz_defaultval;
    points_xyz_defaultval.x = std::numeric_limits<float>::quiet_NaN();
    points_xyz_defaultval.y = std::numeric_limits<float>::quiet_NaN();
    points_xyz_defaultval.z = std::numeric_limits<float>::quiet_NaN();
    output.points.assign(frame_points_num, points_xyz_defaultval);

    for (int i = 0; i < frame_points_num; i++) {
      float range = dst[i].distance / 1000.f;  // distance unit transform to meters
      float intensity = dst[i].intensity;      // laser receive intensity 
      float dir_angle = ANGLE_TO_RADIAN(dst[i].angle);
      //  Transforming polar coordinates into Cartesian coordinates
      output.points[i].x = range * cos(dir_angle);
      output.points[i].y = range * sin(dir_angle);
      output.points[i].z = 0.0;
      output.channels[0].values[i] = intensity;
    }
    lidar_pub_pointcloud->publish(output);
    end_scan_time = start_scan_time;
  }

  ///////////////////////////////////////////////////////////////////////////
    // Members
  ///////////////////////////////////////////////////////////////////////////
  std::thread scan_thread;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_laserscan;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr lidar_pub_pointcloud;

  ldlidar::LDLidarDriverLinuxInterface* ldlidar_drv;

  std::string product_name;
	std::string laser_scan_topic_name;
  std::string point_cloud_2d_topic_name;
	std::string port_name;
  LaserScanSetting setting;
  int serial_baudrate;
  double lidar_scan_freq;
  ldlidar::LDType lidartypename;
  ldlidar::Points2D laser_scan_points;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  std::shared_ptr<LDLidarNode> node;
  try {
      node = std::make_shared<LDLidarNode>();
  } catch (const std::exception &e) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "[LDLIDAR] Failed to start: %s", e.what());
      rclcpp::shutdown();
      return 1;
  }

  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  
  rclcpp::shutdown();
  return 0;
}