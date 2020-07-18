#include "matplotlibcpp.h"
#include <cmath>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

// // splits each line, converts to float and populates the vector
void split_and_assign(const std::string& line, char delimiter, float* x_z_yaw_pair) {
    std::vector<float> split_values;
    std::string single_value;
    std::istringstream lineStream(line);

    int val_idx = 0;
    while( std::getline(lineStream, single_value, delimiter) ) {
        x_z_yaw_pair[val_idx] = std::stof(single_value);
        val_idx++;
    }
}

int main()
{

    // read lidar measurement values
    std::ifstream wheel_tick_file("../wheel_measurement.txt");
    std::ifstream ground_truth_file("../ground_truth_wheels.txt");
    std::string line_wheel;
    std::string line_gt;

    int ctr=0;

    std::vector<float> x, z, x_gt, z_gt;
    float x_z_yaw_pair[3];

    // set figure properties
    matplotlibcpp::figure_size(600,400);

    while(std::getline(wheel_tick_file, line_wheel)) {
        std::getline(ground_truth_file, line_gt); // should have the same length as wheel measurement file
        split_and_assign(line_wheel, ' ', x_z_yaw_pair);
        x.push_back(x_z_yaw_pair[0]);
        z.push_back(x_z_yaw_pair[1]);
        split_and_assign(line_gt, ' ', x_z_yaw_pair);
        x_gt.push_back(x_z_yaw_pair[0]);
        z_gt.push_back(x_z_yaw_pair[1]);

        matplotlibcpp::clf();
        matplotlibcpp::named_plot("wheel odometry", x,z);
        matplotlibcpp::named_plot("ground truth", x_gt, z_gt, "r");
        matplotlibcpp::title("Turtlebot Movement");
        matplotlibcpp::grid(true);
        matplotlibcpp::legend();        
        matplotlibcpp::pause(0.01);

        if(!ctr) {getchar();ctr++;}
    }

    // matplotlibcpp::named_plot("wheel odometry", x,z);
    // matplotlibcpp::named_plot("ground truth", x_gt, z_gt, "r");
    // matplotlibcpp::title("Turtlebot Movement");
    // matplotlibcpp::grid(true);
    // matplotlibcpp::legend();
    // matplotlibcpp::save("../wheel_odometry.png");
    // matplotlibcpp::show();   
}

