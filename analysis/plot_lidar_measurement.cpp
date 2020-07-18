#include "matplotlibcpp.h"
#include <cmath>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <math.h>

// splits each line, converts to float and populates the vector
void split_and_assign(const std::string& line, char delimiter, std::vector<float>& depth_values) {
    std::vector<float> split_values;
    std::string single_value;
    std::istringstream lineStream(line);

    int val_idx = 0;
    while( std::getline(lineStream, single_value, delimiter) ) {
        depth_values[val_idx] = std::stof(single_value);
        val_idx++;
    }
}

// find the derivates in the lidar scan data
void compute_derivative(const std::vector<float>& depth_values, std::vector<float>& depth_deriv) {
    float left_val;
    float right_val;
    int deriv_idx = 0;

    for(auto dept_val_itr = depth_values.begin()+1; dept_val_itr != depth_values.end()-1; dept_val_itr++) {
        left_val = *(dept_val_itr-1);
        right_val = *(dept_val_itr+1);
        depth_deriv[deriv_idx] = (right_val - left_val) / 2.0;
        deriv_idx++;
    }
}

// for each area between a left falling edge and a right rising edge, determine the average ray number and average depth
// strategy: discard the falling edge, which corresponds to the further obstacle
void find_obstacles(const std::vector<float>& depth_values, const std::vector<float>& depth_deriv, 
                    float depth_jump_threshold, std::vector<std::pair<float,float>>& obstacles) {
    bool on_obstacle = false;
    // std::vector<>
    float ray_sum = 0.0; // to keep the average position (index) of the ray
    float depth_sum = 0.0; // to keep the average depth of the detected obstacle
    int rays = 0; // to count the number of rays hitting the closest obstacle

    for(auto depth_deriv_itr = depth_deriv.begin(); depth_deriv_itr != depth_deriv.end(); depth_deriv_itr++) {
        if(*depth_deriv_itr < -1.0*depth_jump_threshold) {
            on_obstacle = true; // in case another obstacle comes before moving away from this one (occluded obstacle)
            rays = 0;
            ray_sum = 0.0;
            depth_sum = 0.0;
        }
        else if ( (*depth_deriv_itr > depth_jump_threshold) && on_obstacle) { // last value within the obstacle area
            on_obstacle = false; // we dont need to keep track of this osbtacle anymore
            rays++;
            ray_sum += depth_deriv_itr - depth_deriv.begin() ;
            depth_sum += depth_values[depth_deriv_itr - depth_deriv.begin()+1]; // since derivative have offset of 1 from compute_derivative function, we add +1

            obstacles.push_back(std::make_pair(ray_sum/rays, depth_sum/rays));
        }

        // area between falling and rising edge ---> obstacle
        if(on_obstacle) {
            rays++;
            ray_sum += depth_deriv_itr - depth_deriv.begin();
            depth_sum += depth_values[depth_deriv_itr - depth_deriv.begin()+1];
        }
    }
}

float index_to_angle(const float& index){
    return (index - 360.0) * M_PI / 180;
}

void compute_cartesian_obstacle(const float& cylinder_radius, std::vector<std::pair<float,float>>& obstacles,
                        std::vector<std::pair<float,float>>& obstacle_positions) {
    
    float obstacle_dist;
    float obstacle_angle;

    for(auto obstacle_itr = obstacles.begin(); obstacle_itr != obstacles.end(); obstacle_itr++)  {
        
        obstacle_angle = index_to_angle(obstacle_itr->first);
        // this is a crude approximation since our depth value is an average of all the rays that hit the cylinder, not just the shortest distance
        obstacle_dist = obstacle_itr->second + cylinder_radius;
        obstacle_positions[obstacle_itr-obstacles.begin()] = std::make_pair(obstacle_dist*cos(obstacle_angle), obstacle_dist*sin(obstacle_angle));   
    }
}

void compute_cartesian_ray(const std::vector<float>& depth_values, std::vector<std::pair<float,float>>& ray_coordinates) {
    float ray_angle;
    int idx;

    for(auto depth_itr = depth_values.begin(); depth_itr != depth_values.end(); depth_itr++) {
        idx = depth_itr-depth_values.begin();
        ray_angle = index_to_angle(idx);
        ray_coordinates[idx] = std::make_pair( (*depth_itr)*cos(ray_angle), (*depth_itr)*sin(ray_angle));   
    }
}

void plot_depth_and_derivatives(const std::map<std::string, std::string>& scatter_args,
                                const std::vector<float>& depth_values,
                                const std::vector<float>& depth_deriv,
                                const std::vector<float>& average_indices,
                                const std::vector<float>& average_depths) {
    
    // plot depth and obstacles
    matplotlibcpp::clf();
    matplotlibcpp::named_plot("depth values", depth_values);
    matplotlibcpp::named_plot("depth derivatives", depth_deriv, "r");
    matplotlibcpp::title("Lidar Measurement");
    matplotlibcpp::scatter(average_indices,average_depths,30, scatter_args);
    matplotlibcpp::legend();
    matplotlibcpp::grid(true);
    matplotlibcpp::pause(0.01);
}

void plot_point_cloud(const std::map<std::string, std::string>& scatter_args2, 
                      const std::map<std::string, std::string>& scatter_args3,
                      const std::vector<float>& obstacle_x, const std::vector<float>& obstacle_y,
                      const std::vector<float>& ray_x, const std::vector<float>& ray_y) {
                          
    // plot position of the obstacles
    matplotlibcpp::clf();
    matplotlibcpp::title("Lidar Measurement");

    matplotlibcpp::scatter(obstacle_x, obstacle_y, 60, scatter_args2);
    matplotlibcpp::legend();

    matplotlibcpp::scatter(ray_x, ray_y, 7, scatter_args3);
    matplotlibcpp::legend();
    matplotlibcpp::grid(true);
    
    matplotlibcpp::pause(0.01);   
}


int main()
{
    // read lidar measurement values
    std::ifstream lidar_file("../lidar_measurement.txt");
    std::string line;

    int lidar_resolution = 360;
    float depth_jump_threshold = 0.02; // tune till 2ndary peaks are not detected
    float cylinder_radius = 0.05;
    int ctr=0;

    // initialize a vector to hold the lidar data at each timestep
    std::vector<float> depth_values(lidar_resolution); // initializes all the values to 0
    // container for cartesian coordinates of each ray hit
    std::vector<std::pair<float,float>> ray_coordinates(lidar_resolution); // initializes all the values to 0
    // container for scan derivatives
    std::vector<float> depth_deriv(lidar_resolution-2); // we cannot use first and last values due to overflow of index

    // functors to extract the first and second element of the pair
    auto get_first = [](const std::pair<float, float>& pair_data) {return pair_data.first;};
    auto get_second = [](const std::pair<float, float>& pair_data) {return pair_data.second;};

    // set plotting properties
    matplotlibcpp::figure_size(1000,800);
    std::map<std::string, std::string> scatter_args;
    scatter_args.insert(std::make_pair("c","g"));
    scatter_args.insert(std::make_pair("label","detected obstacle"));   
    std::map<std::string, std::string> scatter_args2;
    scatter_args2.insert(std::make_pair("c","r"));
    scatter_args2.insert(std::make_pair("label","detected obstacle"));    
    std::map<std::string, std::string> scatter_args3;
    scatter_args3.insert(std::make_pair("c","b"));
    scatter_args3.insert(std::make_pair("label","rays"));     

    while(std::getline(lidar_file, line)) { // each getline is 1 whole scan
        // container for the detected obstacles per scan (average ray index, average depth)
        std::vector<std::pair<float,float>> obstacles;
               
        split_and_assign(line, ' ', depth_values);

        compute_derivative(depth_values, depth_deriv);

        find_obstacles(depth_values, depth_deriv, depth_jump_threshold, obstacles);

        std::vector<std::pair<float,float>> obstacle_positions(obstacles.size()); // find cartesian coordinates of the detected obstacle
        compute_cartesian_obstacle(cylinder_radius, obstacles, obstacle_positions);

        compute_cartesian_ray(depth_values, ray_coordinates);

        // now allocate memory for the average index and average depth arrays, since the size of obstacles is known now
        std::vector<float> average_indices(obstacles.size());
        std::vector<float> average_depths(obstacles.size());
        
        std::vector<float> obstacle_x(obstacles.size());
        std::vector<float> obstacle_y(obstacles.size());

        std::vector<float> ray_x(lidar_resolution);
        std::vector<float> ray_y(lidar_resolution);        

        std::cout << "# of obstacles:" << obstacles.size() << std::endl;

        // extract all first elements (indices) from the vector of pairs
        std::transform(begin(obstacles), end(obstacles), std::back_inserter(average_indices), get_first);
        std::transform(begin(obstacles), end(obstacles), std::back_inserter(average_depths), get_second);

        std::transform(begin(obstacle_positions), end(obstacle_positions), std::back_inserter(obstacle_x), get_first);
        std::transform(begin(obstacle_positions), end(obstacle_positions), std::back_inserter(obstacle_y), get_second);  

        std::transform(begin(ray_coordinates), end(ray_coordinates), std::back_inserter(ray_x), get_first);
        std::transform(begin(ray_coordinates), end(ray_coordinates), std::back_inserter(ray_y), get_second);                     

        // plot_depth_and_derivatives(scatter_args, depth_values, depth_deriv, average_indices, average_depths);

        plot_point_cloud( scatter_args2, scatter_args3, obstacle_x, obstacle_y, ray_x, ray_y);

        if(!ctr) {getchar();ctr++;}
        
    }
}