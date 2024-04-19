/*
 * File:          stick_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/supervisor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 32
#define JOINT_NUMBER 6
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
enum State { MOVE_BOX, WAIT, ROTATING, RELEASING, ROTATING_BACK };


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
    
  wb_robot_init();
  WbNodeRef belt_node = wb_supervisor_node_get_from_def("conveyor_belt");
  WbFieldRef belt_speed = wb_supervisor_node_get_field(belt_node, "speed");
  
  WbDeviceTag dist_sensor_1 = wb_robot_get_device("box_trigger_1");
  WbDeviceTag dist_sensor_2 = wb_robot_get_device("box_trigger_2");
  
  wb_distance_sensor_enable(dist_sensor_1, TIME_STEP);
  wb_distance_sensor_enable(dist_sensor_2, TIME_STEP);
  
  
  // WbDeviceTag robot_motors[6];
  // char *joint_names[6] = {"pr15_shoulder_pan_joint","pr15_shoulder_lift_joint","pr15_elbow_joint","pr15_wrist1_joint","pr15_wrist2_joint","pr15_wrist3_joint"};
  
  // for (int i = 0; i != JOINT_NUMBER; i++)
  // {
    // robot_motors[i] = wb_robot_get_device(joint_names[i]);
    // wb_motor_set_velocity(robot_motors[i], 1.57);
  // }
  

  wb_supervisor_field_set_sf_float(belt_speed, 0.25);
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
  int state = MOVE_BOX;
  while (wb_robot_step(TIME_STEP) != -1) {
     double val1 = wb_distance_sensor_get_value(dist_sensor_1);
     double val2 = wb_distance_sensor_get_value(dist_sensor_2);
     
     switch(state)
     {
       case MOVE_BOX:
         if (wb_distance_sensor_get_value(dist_sensor_1) < 420 && wb_distance_sensor_get_value(dist_sensor_2) < 420)
         {
           wb_supervisor_field_set_sf_float(belt_speed, 0.0);
           state = WAIT;
         }
         break;
       
       case WAIT:
         wb_supervisor_field_set_sf_float(belt_speed, 0.25);
         state = MOVE_BOX;
         break;
               
     }
     
     // if (val1 < 420 && val2 < 420)
     // {
       // wb_supervisor_field_set_sf_float(belt_speed, 0.0);
     // }
     
     // else 
     // {
       // wb_supervisor_field_set_sf_float(belt_speed, 0.25);
     // }
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
