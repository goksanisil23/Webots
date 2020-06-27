#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/motor.h>

#define TIME_STEP 32

static int my_exit(FILE* measurement_file_ptr, FILE* ground_truth_file_ptr) {
  wb_robot_cleanup();
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
  wb_robot_init();

  // setup logging
  FILE* measurement_file_ptr;
  FILE* ground_truth_file_ptr;
  measurement_file_ptr = fopen("/home/gisil/Work/SLAM/analysis/measurement.txt", "w+");
  ground_truth_file_ptr = fopen("/home/gisil/Work/SLAM/analysis/ground_truth.txt", "w+");

  // get handle to robot's translation field
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_TURTLE");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation"); // this is quaternion

  // get handle to the robot's wheel motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // get handle to the robot's wheel sensors
  WbDeviceTag left_wheel_sensor = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_wheel_sensor = wb_robot_get_device("right wheel sensor");
  // get handle to inertial sensor 
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");

  // need to enable sensors before using
  wb_position_sensor_enable(left_wheel_sensor, TIME_STEP);
  wb_position_sensor_enable(right_wheel_sensor, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);

  // set position to infinity since we want velocity control
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  double left_wheel_sensor_val;
  double right_wheel_sensor_val;
  double left_wheel_sensor_val_old;
  double right_wheel_sensor_val_old;
  double delta_right_wheel;
  double delta_left_wheel;

  double first_z = 0;
  double first_x = 0;

  // allocate memory for new_pose and old_pose
  const double* translation_values = wb_supervisor_field_get_sf_vec3f(trans_field);
  const double* quat_rotation_values = wb_supervisor_field_get_sf_rotation(rotation_field);
  double euler_rotation_values[3] = {0, 0, 0};
  const double* roll_pitch_yaw;

  // pose = [longitude, lateral]
  double old_pose[3];
  double new_pose[3];
  
  // run for a certain duration (in simulation time)
  double t = wb_robot_get_time();
  int ctr = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // read sensor outputs

    roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("%f\n", roll_pitch_yaw[2]*180/M_PI);


    if(ctr == 0) {
      left_wheel_sensor_val_old =  wb_position_sensor_get_value(left_wheel_sensor);     
      right_wheel_sensor_val_old = wb_position_sensor_get_value(right_wheel_sensor);

      translation_values = wb_supervisor_field_get_sf_vec3f(trans_field);
      first_z = translation_values[2];
      first_x = translation_values[0];

      old_pose[0] = translation_values[0];
      old_pose[1] = translation_values[2]; 
      old_pose[2] = roll_pitch_yaw[2];
      memcpy(new_pose, old_pose, sizeof(old_pose));      

    }
    else {
      left_wheel_sensor_val = wb_position_sensor_get_value(left_wheel_sensor);
      right_wheel_sensor_val = wb_position_sensor_get_value(right_wheel_sensor);    

      delta_right_wheel = right_wheel_sensor_val - right_wheel_sensor_val_old;
      delta_left_wheel = left_wheel_sensor_val - left_wheel_sensor_val_old;

      left_wheel_sensor_val_old = left_wheel_sensor_val;
      right_wheel_sensor_val_old = right_wheel_sensor_val;
      
      // process behavior
      filter_step(old_pose, new_pose, delta_left_wheel, delta_right_wheel);
      fprintf(measurement_file_ptr, "%f %f %f\n", new_pose[0], new_pose[1], new_pose[2]);
      memcpy(old_pose, new_pose, sizeof(old_pose));
      printf("old pose yaw: %f\n", old_pose[2]);

      translation_values = wb_supervisor_field_get_sf_vec3f(trans_field);
      fprintf(ground_truth_file_ptr, "%f %f\n", translation_values[0], translation_values[2]);      
    }
    
    
    // write actuator 
    // if (wb_robot_get_time() - t < 5.0) {
    //  wb_motor_set_velocity(left_motor, 3.0);
    //  wb_motor_set_velocity(right_motor, 2.5);       
    // }
    // else if (wb_robot_get_time() - t < 10.0) {
    //  wb_motor_set_velocity(left_motor, 1.0);
    //  wb_motor_set_velocity(right_motor, 2.0);          
    // }
    wb_motor_set_velocity(left_motor, 3.0);
    wb_motor_set_velocity(right_motor, 3.0);
    
    // controller termination based on condition
    if (wb_robot_get_time() - t > 10.0) {
      break;
    }

    ctr++;
  }

  // compute travelled distance
  translation_values = wb_supervisor_field_get_sf_vec3f(trans_field);
  double dist = sqrt((translation_values[0]-first_x) * (translation_values[0]-first_x) + (translation_values[2]-first_z) * (translation_values[2]-first_z));
  
  printf("x = %g\n", translation_values[0]);
  printf("z = %g\n", translation_values[2]);
  printf("dist traveled = %g\n", dist);
  printf("left_wheel_sensor_val = %g\n", left_wheel_sensor_val);
  printf("right_wheel_sensor_val = %g\n", right_wheel_sensor_val);

  

  // reset robot position and physics
  // const double INITIAL[3] = { 0, 0, 0 };
  // wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
  // wb_supervisor_node_reset_physics(robot_node);

  // end of tests
  return my_exit(measurement_file_ptr, ground_truth_file_ptr);
}