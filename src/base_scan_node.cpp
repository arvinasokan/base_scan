/* 
   Copyright Information
*/

// Author: Arvin Asokan

// Standard C headers
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <std_msgs/String.h>

// ROS message headers
#include <sensor_msgs/LaserScan.h>

// C++ headers
#include <iostream>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>


// Laser Scanner Class
class LaserScanner{
 private:
    // All the necessary declarations
    double angle_min_, angle_max_, angle_increment_;
    double time_increment_, range_min_, range_max_;
    ros::NodeHandle nh;
    ros::Publisher laser_scan_pub;
    int num_readings_;
    std::vector<int> ranges, intensities;
    double laser_frequency_;
    ros::Time scan_time_;

 public:
LaserScanner() {
  // The published topic is called scan as this is the 
  // topic to which many of the ROS Navigation and mapping packages subscribe to by default.
  // eg.AMCL, gmapping, etc.
  laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
  // All the necessary parameters have to be set in the config file.
  // All the default values are based on SICK TIM 571 LIDAR which is being used in the Fetches and Freights

  if (!nh.getParam("/base_scan_node/laser_frequency", laser_frequency_)) {
      laser_frequency_ = 15;
      ROS_WARN("laser frequency has not been set, using the default value instead : %f", laser_frequency_);
    }

  if (!nh.getParam("/base_scan_node/angle_min", angle_min_)) {
      angle_min_ = -1.91986;
      ROS_WARN("Minimum angle has not been set, using the default value instead : %f", angle_min_);
    }
    
  if (!nh.getParam("/base_scan_node/angle_max", angle_max_)) {
      angle_max_ = 1.91986;
      ROS_WARN("Maximum angle has not been set, using the default value instead : %f", angle_min_);
    }

  if (!nh.getParam("/base_scan_node/range_min", range_min_)) {
      range_min_ = 0;
      ROS_WARN("Minimum range has not been set, using the default value instead : %f", range_min_);
    }

  if (!nh.getParam("/base_scan_node/range_max", range_max_)) {
      range_max_ = 25;
      ROS_WARN("Maximum range has not been set, using the default value instead : %f", range_max_);
    }

  if (!nh.getParam("/base_scan_node/angle_increment", angle_increment_)) {
      angle_increment_ = 0.005759587;
      ROS_WARN("Angular resolution has not been set, using the default value instead : %f", angle_increment_);
    }

  num_readings_ =  static_cast<int>((std::abs(angle_max_ - angle_min_) / angle_increment_));
  time_increment_ = (1 / laser_frequency_) / (num_readings_);
  ranges.resize(num_readings_);
  intensities.resize(num_readings_);
  ranges.clear();
  intensities.clear();
}
~LaserScanner() {
}
// This function is unnecessary ,
// as the fake ranges can be generated and filled in the final loop ,
// I am keeping this just to keep it separate.
void generate_laser_scans() {
  srand(time(NULL));
  // Code Frequency in hz
  ros::Rate r(50);
  while (nh.ok()) {
  // Just to show some variation in values
  for (unsigned int i = 0; i < num_readings_/3 ; ++i) {
      ranges.push_back(5);
    }
  for (unsigned int i = num_readings_/3; i < (num_readings_/3)*2 ; ++i) {
      ranges.push_back(5);
    }
  for (unsigned int i = (num_readings_/3)*2; i < num_readings_ ; ++i) {
      ranges.push_back(5);
    }
  scan_time_ = ros::Time::now();
  publish_scan();
  r.sleep();
}
}
bool publish_scan() {
    // Packing all the sensor data
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time_;
    scan.header.frame_id = "laser_link";
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = time_increment_;
    scan.scan_time = 1 / laser_frequency_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.resize(num_readings_);
    scan.intensities.resize(num_readings_);
    // Filling the ranges
    // Leaving the intensities blank as it may vary from device to device.
    for (unsigned int i = 0; i < num_readings_; ++i) {
      scan.ranges[i] = ranges[i];
    }
    ranges.clear();
    // Publishing the scan data
    laser_scan_pub.publish(scan);
    return true;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_scan");
  LaserScanner ls;
  ls.generate_laser_scans();
  ros::spin();
  return 0;
}
