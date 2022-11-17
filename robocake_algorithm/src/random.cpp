#include "algorithm.h"

#include <random>

int direction{0};

geometry_msgs::Twist control(
                     std::unordered_map<std::string, double> proximity,
                     std::unordered_map<std::string, double> reflectance,
                     double threshold) {
  std::uniform_real_distribution<> d{0, 1};
  std::random_device r;
  std::default_random_engine g(r());
  geometry_msgs::Twist msg;
  if (proximity["front"] > 0.1 && proximity["left"] > 0.1 &&
      proximity["right"] > 0.1 &&
      reflectance["left"] > threshold && reflectance["right"] > threshold) {
    direction = 0;
    msg.linear.x = 0.2;
    msg.angular.z = 0;
  } else {
    if (!direction) {
      if (proximity["right"] < 0.1 || reflectance["right"] < threshold) {
        direction = 1;
      } else if (proximity["left"] < 0.1 || reflectance["left"] < threshold) {
        direction = -1;
      } else {
        direction = d(g) < 0.5 ? 1 : -1;
      }
      if (d(g) < 0.2) {
        direction *= -1;
      }
    }
    msg.linear.x = 0;
    msg.angular.z = direction * M_PI / 2;

  }
  return msg;
}
