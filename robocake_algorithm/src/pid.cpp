#include "algorithm.h"
#include <fstream>
float lastError = 0;
int intCumulation = 0;
float maxCumulation = 10;
float p = 5;
float i = 4;
float d = 6;

float pid(std::unordered_map<std::string, double> reflectance) {
    float error = -1*(reflectance["left"] - reflectance["right"]);
    float signal = p * lastError + i * intCumulation + d * (error - lastError);
    intCumulation = std::abs(intCumulation + error) < maxCumulation ? intCumulation + error : maxCumulation;
    lastError = error;
    return signal;
}

geometry_msgs::Twist control(
        std::unordered_map<std::string, double> proximity,
        std::unordered_map<std::string, double> reflectance,
        double threshold) {
    geometry_msgs::Twist msg;
    float pid_direction = pid(reflectance);
    double distance = -1;
    float prox_direction = 0;
    if (proximity["front"] < 0.1) {
        distance = proximity["front"];
        prox_direction = 1;
    } else if (proximity["left"] < 0.1) {
        distance = proximity["left"];
        prox_direction = 1;
    } else if (proximity["right"] < 0.1) {
        distance = proximity["right"];
        prox_direction = -1;
    }
    /*if (distance > 0) {
        auto v = std::min(std::abs(distance - threshold) / 0.1, 1.0) * 0.25;
        msg.linear.x = distance < threshold ? -v : v;
        msg.angular.z = pid_direction;
    } else {
        msg.linear.x = 0.3;
        msg.angular.z = pid_direction;
    }*/
    if (proximity["front"]>0.7 && std::abs(pid_direction)<0.6){
        msg.linear.x = 0.25;
    }else{
        msg.linear.x = 0.2;
    }

    msg.angular.z = pid_direction;
    /*std::ofstream out;
    out.open("~/RosProjects/robocake/src/beginner/robocake_algorithm/src/log.txt", std::ios_base::app);
    if(!out){
        out<<msg.angular.z << " " << reflectance["left"] << " " << reflectance["right"] << '\n';
    }
    out.close();*/
    //std::cout << msg.angular.z << " " << reflectance["left"] << " " << reflectance["right"] << '\n';
    return msg;
}
