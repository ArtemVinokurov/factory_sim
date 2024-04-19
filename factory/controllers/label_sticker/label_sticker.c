/*
 * File:          label_sticker.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
/*
 * You may want to add macros here.
 */


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

#include <webots/robot.h>
#include <webots/utils/motion.h>
#define TIME_STEP 32
#define JOINT_NUMBER 6
int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  // load and start forward motion

  // forever
  // bool reverse = false;
  // while (1) {
    // reverse = reverse ? false : true;
    // wbu_motion_set_reverse(motion, reverse);
  WbDeviceTag robot_motors[6];
  char *joint_names[6] = {"pr15_shoulder_pan_joint","pr15_shoulder_lift_joint","pr15_elbow_joint","pr15_wrist1_joint","pr15_wrist2_joint","pr15_wrist3_joint"};
  
  for (int i = 0; i != JOINT_NUMBER; i++)
  {
    robot_motors[i] = wb_robot_get_device(joint_names[i]);
    wb_motor_set_velocity(robot_motors[i], 1.57);
  }
  while (wb_robot_step(TIME_STEP) != -1) 
  {
     wb_robot_set_custom_data("start");
     double pos1[6] = {0.0};
     double pos2[6] = {-1.57,0,1.57,0,1.57,0};
     // for (int i = 0; i != JOINT_NUMBER; i++)
     // {
       // wb_motor_set_position(robot_motors[i], pos1[i]);
     // }
   WbMotionRef motion = wbu_motion_new("puma560.motion");
   wbu_motion_play(motion); 
     // for (int i = 0; i != JOINT_NUMBER; i++)
     // {
       // wb_motor_set_position(robot_motors[i], pos2[i]);
     // }
     // wb_robot_set_custom_data("done");
  }
  // never reached
  return 0;
}

