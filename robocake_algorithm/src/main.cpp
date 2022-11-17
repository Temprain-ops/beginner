#include "algorithm.h"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

std::unordered_map<std::string, double> proximity, reflectance;
ros::Publisher publisher;

void update_proximity(std::string direction, const sensor_msgs::Range& msg) {
  proximity[direction] = msg.range;
}

void update_reflectance(std::string direction, const std_msgs::Float64& msg) {
  reflectance[direction] = msg.data;
}

double threshold;

void timer_callback(const ros::TimerEvent&) {
  publisher.publish(control(proximity, reflectance, threshold));
}

template <class M> void subscribe(std::vector<ros::Subscriber> &subscribers,
                                  std::vector<std::string> directions,
                                  void (*update)(std::string, const M&),
                                  std::string prefix) {
  ros::NodeHandle node;
  for (auto direction : directions) {
    auto topic{prefix + "_" + direction};
    auto callback{boost::bind(update, direction, _1)};
    auto subscriber = node.subscribe<M, const M&>(topic, 1, callback);
    subscribers.emplace_back(subscriber);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robocake_random");
  std::vector<ros::Subscriber> subscribers;
  ros::NodeHandle node;

  subscribe(subscribers, {"left", "right", "front"}, update_proximity,
            "proximity");
  subscribe(subscribers, {"left", "right", "center"}, update_reflectance,
            "reflectance");

  publisher = node.advertise<geometry_msgs::Twist>(
      "diff_drive_controller/cmd_vel", 1);

  threshold = ros::NodeHandle{"~"}.param("threshold", 0.5);

  auto timer = node.createTimer(ros::Duration{0.1}, timer_callback);

  ros::spin();
}
