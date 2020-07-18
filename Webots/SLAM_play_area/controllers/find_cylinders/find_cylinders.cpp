#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>

#include <stdio.h>
#include <math.h>
#include <cstring>

#include <thread>
#include <chrono>

#define TIME_STEP 32

// using namespace webots;

static int cleanUp(FILE* measurement_file_ptr, FILE* ground_truth_file_ptr, FILE* lidar_file_ptr, webots::Supervisor* robot) {

  robot->simulationReset();
  robot->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
  robot->simulationResetPhysics();
  // robot->restartController();

  delete robot;

  fclose(measurement_file_ptr);
  fclose(ground_truth_file_ptr);
  fclose(lidar_file_ptr);

  return 0;
}

// static robot parameters
double wheel_radius = 0.033;
double wheel_base = 0.16;
double lidar_z_offset = 0.03; // obtained from the .proto file's children section for extension slot

double constrainAngle(double angle){
    // angle = fmod(angle,360);
    // if (angle < 0)
    //     angle += 360;
    return angle;
}

void filter_step(double* old_pose, double* new_pose, double left_wheel_sensor_val, double right_wheel_sensor_val) {
  
  // compute the arclength, from the radians measurement of the encoder sensor
  double l = left_wheel_sensor_val * wheel_radius;
  double r = right_wheel_sensor_val * wheel_radius;

  // if there is a turn at all
  if(left_wheel_sensor_val == right_wheel_sensor_val) {
    double theta = old_pose[2];
    double x = old_pose[0] + (l * cos(theta));
    double z = old_pose[1] - (l * sin(theta));

    new_pose[0] = x; 
    new_pose[1] = z;
    new_pose[2] = theta;

  }
  else {
    // First, modify the old_pose to get the robot center, since the old_pose is actually Lidar's pose
    double theta = old_pose[2];
    double x = old_pose[0] - lidar_z_offset * cos(theta);
    double z = old_pose[1] - lidar_z_offset * sin(theta);

    // Second, execute the odometry code that implements the motion model for the center of the robot
    double alpha = (r - l) / wheel_base;
    double R = l / alpha;
    double cx = x - (R + wheel_base/2) * sin(theta);
    double cz = z - (R + wheel_base/2) * cos(theta);
    theta = fmod(theta + alpha, 2*M_PI);
    
    x = cx + (R + wheel_base / 2) * sin(theta);
    z = cz + (R + wheel_base / 2) * cos(theta);    

    // Third, modify the result to get back the Lidar pose from the computed robot center. This is the value we return
    x = x + lidar_z_offset * cos(theta);
    z = z + lidar_z_offset * sin(theta);

    new_pose[0] = x; 
    new_pose[1] = z;
    new_pose[2] = theta;

  }

}

int main() {
  // create a robot instance
  webots::Supervisor* robot = new webots::Supervisor(); // initializes both robot and supervisor

  // setup logging
  FILE* measurement_file_ptr;
  FILE* ground_truth_file_ptr;
  FILE* lidar_file_ptr;
  measurement_file_ptr = fopen("/home/gisil/Work/SLAM/SLAM/analysis/wheel_measurement.txt", "w+");
  ground_truth_file_ptr = fopen("/home/gisil/Work/SLAM/SLAM/analysis/ground_truth_wheels.txt", "w+");
  lidar_file_ptr = fopen("/home/gisil/Work/SLAM/SLAM/analysis/lidar_measurement.txt", "w+");

  // get handle to robot's translation field
  webots::Node* robot_node = robot->getFromDef("MY_TURTLE");
  webots::Field* trans_field = robot_node->getField("translation"); // ground truth
  webots::Field* rotation_field = robot_node->getField("rotation"); // ground truth (quaternion)

  // get handle to the robot's wheel motors
  webots::Motor* left_motor = robot->getMotor("left wheel motor");
  webots::Motor* right_motor = robot->getMotor("right wheel motor");

  // get handle to lidar's motors
  webots::Motor* lidar_main_motor = robot->getMotor("LDS-01_main_motor");
  webots::Motor* lidar_secondary_motor = robot->getMotor("LDS-01_secondary_motor");

  // get handle to the robot's sensors
  webots::PositionSensor* left_wheel_sensor = robot->getPositionSensor("left wheel sensor");
  webots::PositionSensor* right_wheel_sensor = robot->getPositionSensor("right wheel sensor");
  webots::InertialUnit* inertial_unit = robot->getInertialUnit("inertial unit");
  webots::Lidar* lidar = robot->getLidar("LDS-01");

  // need to enable sensors before using
  left_wheel_sensor->enable(TIME_STEP);
  right_wheel_sensor->enable(TIME_STEP);
  inertial_unit->enable(TIME_STEP);
  lidar->enable(TIME_STEP);
  lidar->enablePointCloud();

  // set position to infinity since we want velocity control
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  // run the lidar motors (just for visualization, no effect on sensor reading)
  lidar_main_motor->setPosition(INFINITY);
  lidar_secondary_motor->setPosition(INFINITY);
  lidar_main_motor->setVelocity(30.0);
  lidar_secondary_motor->setVelocity(60.0);  

  double left_wheel_sensor_val;
  double right_wheel_sensor_val;
  double left_wheel_sensor_val_old;
  double right_wheel_sensor_val_old;
  double delta_right_wheel;
  double delta_left_wheel;

  double first_z = 0;
  double first_x = 0;

  // allocate memory for new_pose and old_pose
  const double* translation_values = trans_field->getSFVec3f();
  const double* quat_rotation_values = rotation_field->getSFRotation();
  double euler_rotation_values[3] = {0, 0, 0};
  const double* roll_pitch_yaw;
  // allocate memory for the pointer to the lidar value array (pointer is on stack but the actual values are on the heap since its dynamically allocated)
  const float* lidar_values;
  // we need to keep track of the lidar's return size like this since it's allocated on the heap dynamically by webots
  const int lidar_data_length = lidar->getHorizontalResolution()* lidar->getNumberOfLayers(); 

  // pose = [x, z, yaw_angle]
  double old_pose[3];
  double new_pose[3];
  
  // run for a certain duration (in simulation time)
  double t = robot->getTime();
  int ctr = 0;

  while (robot->step(TIME_STEP) != -1) {
    
    // read sensor outputs
    roll_pitch_yaw = inertial_unit->getRollPitchYaw();
    printf("yaw angle: %f\n", roll_pitch_yaw[2]*180/M_PI);

    if(ctr == 0) {
      left_wheel_sensor_val_old =  left_wheel_sensor->getValue();     
      right_wheel_sensor_val_old = right_wheel_sensor->getValue();

      translation_values = trans_field->getSFVec3f();
      first_z = translation_values[2];
      first_x = translation_values[0];

      // assuming initial values are "known": Note that we are tracking lidar's position here, to make it easier to relate to measurements
      old_pose[0] = translation_values[0] + lidar_z_offset * cos(roll_pitch_yaw[2]) ; // this value needs to be lidar's value, so we offset from turtlebot's position
      old_pose[1] = translation_values[2] + lidar_z_offset * sin(roll_pitch_yaw[2]); 
      old_pose[2] = roll_pitch_yaw[2];
      memcpy(new_pose, old_pose, sizeof(old_pose));      

    }
    else { // after 1st step, robot odometry is tracked only by the "wheel ticks"
      left_wheel_sensor_val = left_wheel_sensor->getValue();
      right_wheel_sensor_val = right_wheel_sensor->getValue();    

      delta_right_wheel = right_wheel_sensor_val - right_wheel_sensor_val_old;
      delta_left_wheel = left_wheel_sensor_val - left_wheel_sensor_val_old;

      left_wheel_sensor_val_old = left_wheel_sensor_val;
      right_wheel_sensor_val_old = right_wheel_sensor_val;
      
      // process behavior
      filter_step(old_pose, new_pose, delta_left_wheel, delta_right_wheel); // delta represents the "wheel tick" since last measurement
      fprintf(measurement_file_ptr, "%f %f %f\n", new_pose[0], new_pose[1], new_pose[2]);
      memcpy(old_pose, new_pose, sizeof(old_pose));

      translation_values = trans_field->getSFVec3f();
      // note that reported ground truth is for the lidar positin =  GT turtlebot position + lidar offset
      fprintf(ground_truth_file_ptr, "%f %f %f\n", translation_values[0]+lidar_z_offset*cos(roll_pitch_yaw[2]), translation_values[2]+lidar_z_offset*sin(roll_pitch_yaw[2]), roll_pitch_yaw[2]);      

      // get & save Lidar measuremenet
      lidar_values = lidar->getRangeImage();
      for(int jj=0; jj<lidar_data_length; jj++) {
        fprintf(lidar_file_ptr, "%f ", lidar_values[jj]);
      }
      fprintf(lidar_file_ptr, "\n");

    }
    
    
    // write actuator 
    if (robot->getTime() - t < 10.0) {
     left_motor->setVelocity(6.0);
     right_motor->setVelocity(3.0);       
    }
    else if (robot->getTime() - t < 15.0) {
     left_motor->setVelocity(2.0);
     right_motor->setVelocity(4.0);          
    }
    else if (robot->getTime() - t < 25.0) {
     left_motor->setVelocity(6.5);
     right_motor->setVelocity(4.0);          
    }    

    // left_motor->setVelocity(3.0);
    // right_motor->setVelocity(3.5);
    
    // controller termination based on condition
    if (robot->getTime() - t > 25.0) {
      break;
    }

    ctr++;
  }

  // compute travelled distance
  translation_values = trans_field->getSFVec3f();
  
  printf("x = %g\n", translation_values[0]);
  printf("z = %g\n", translation_values[2]);
  printf("left_wheel_sensor_val = %g\n", left_wheel_sensor_val);
  printf("right_wheel_sensor_val = %g\n", right_wheel_sensor_val);

  // end of tests
  return cleanUp(measurement_file_ptr, ground_truth_file_ptr, lidar_file_ptr, robot);
}