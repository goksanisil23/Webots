#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>

#include <stdio.h>
#include <math.h>
#include <cstring>

#include <thread>
#include <chrono>

#define TIME_STEP 32

// using namespace webots;

static int cleanUp(FILE* measurement_file_ptr, FILE* ground_truth_file_ptr, webots::Supervisor *robot) {
  delete robot;

  fclose(measurement_file_ptr);
  fclose(ground_truth_file_ptr);

  return 0;
}

double wheel_radius = 0.033;
double wheel_base = 0.16;

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
    printf("l:%f\n", l);

    new_pose[0] = x; 
    new_pose[1] = z;
    new_pose[2] = theta;

  }
  else {
    // Turn. Compute alpha, R, etc.
    double alpha = (r - l) / wheel_base;
    double R = l / alpha;
    double cx = old_pose[0] - (R + wheel_base/2) * sin(old_pose[2]);
    double cz = old_pose[1] - (R + wheel_base/2) * cos(old_pose[2]);
    double theta = fmod(old_pose[2] + alpha, 2*M_PI);
    
    double x = cx + (R + wheel_base / 2) * sin(theta);
    double z = cz + (R + wheel_base / 2) * cos(theta);    

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
  measurement_file_ptr = fopen("/home/gisil/Work/SLAM/SLAM/analysis/wheel_measurement.txt", "w+");
  ground_truth_file_ptr = fopen("/home/gisil/Work/SLAM/SLAM/analysis/ground_truth.txt", "w+");

  // get handle to robot's translation field
  webots::Node* robot_node = robot->getFromDef("MY_TURTLE");
  webots::Field* trans_field = robot_node->getField("translation");
  webots::Field* rotation_field = robot_node->getField("rotation"); // this is quaternion

  // get handle to the robot's wheel motors
  webots::Motor* left_motor = robot->getMotor("left wheel motor");
  webots::Motor* right_motor = robot->getMotor("right wheel motor");

  // get handle to the robot's wheel sensors
  webots::PositionSensor* left_wheel_sensor = robot->getPositionSensor("left wheel sensor");
  webots::PositionSensor* right_wheel_sensor = robot->getPositionSensor("right wheel sensor");
  webots::InertialUnit* inertial_unit = robot->getInertialUnit("inertial unit");

  // need to enable sensors before using
  left_wheel_sensor->enable(TIME_STEP);
  right_wheel_sensor->enable(TIME_STEP);
  inertial_unit->enable(TIME_STEP);

  // set position to infinity since we want velocity control
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

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

  // pose = [longitude, lateral]
  double old_pose[3];
  double new_pose[3];
  
  // run for a certain duration (in simulation time)
  double t = robot->getTime();
  int ctr = 0;

  while (robot->step(TIME_STEP) != -1) {
    
    // read sensor outputs
    roll_pitch_yaw = inertial_unit->getRollPitchYaw();
    printf("%f\n", roll_pitch_yaw[2]*180/M_PI);

    if(ctr == 0) {
      left_wheel_sensor_val_old =  left_wheel_sensor->getValue();     
      right_wheel_sensor_val_old = right_wheel_sensor->getValue();

      translation_values = trans_field->getSFVec3f();
      first_z = translation_values[2];
      first_x = translation_values[0];

      old_pose[0] = translation_values[0];
      old_pose[1] = translation_values[2]; 
      old_pose[2] = roll_pitch_yaw[2];
      memcpy(new_pose, old_pose, sizeof(old_pose));      

    }
    else {
      left_wheel_sensor_val = left_wheel_sensor->getValue();
      right_wheel_sensor_val = right_wheel_sensor->getValue();    

      delta_right_wheel = right_wheel_sensor_val - right_wheel_sensor_val_old;
      delta_left_wheel = left_wheel_sensor_val - left_wheel_sensor_val_old;

      left_wheel_sensor_val_old = left_wheel_sensor_val;
      right_wheel_sensor_val_old = right_wheel_sensor_val;
      
      // process behavior
      filter_step(old_pose, new_pose, delta_left_wheel, delta_right_wheel);
      fprintf(measurement_file_ptr, "%f %f %f\n", new_pose[0], new_pose[1], new_pose[2]);
      memcpy(old_pose, new_pose, sizeof(old_pose));
      printf("old pose yaw: %f\n", old_pose[2]);

      translation_values = trans_field->getSFVec3f();
      fprintf(ground_truth_file_ptr, "%f %f\n", translation_values[0], translation_values[2]);      

    }
    
    
    // write actuator 
    if (robot->getTime() - t < 5.0) {
     left_motor->setVelocity(3.0);
     right_motor->setVelocity(2.5);       
    }
    else if (robot->getTime() - t < 10.0) {
     left_motor->setVelocity(1.0);
     right_motor->setVelocity(2.0);          
    }

    // left_motor->setVelocity(3.0);
    // right_motor->setVelocity(3.5);
    
    // controller termination based on condition
    if (robot->getTime() - t > 10.0) {
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
  return cleanUp(measurement_file_ptr, ground_truth_file_ptr, robot);
}