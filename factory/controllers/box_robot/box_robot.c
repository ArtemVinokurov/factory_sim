#include <webots/robot.h>
#include <webots/utils/motion.h>
#include <webots/vacuum_gripper.h>


void delay(double delay_time)
{
  int time_step = wb_robot_get_basic_time_step();
  int counter = (int)(delay_time * 1000 / time_step);
  while(counter > 0)
  {
    counter -= 1;
    wb_robot_step(time_step);
  }
}


void move_robot(WbMotionRef motion)
{
  wbu_motion_play(motion);
  int time_step = wb_robot_get_basic_time_step();
  while (!wbu_motion_is_over(motion))
      wb_robot_step(time_step);
}


int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag vacuum_gripper = wb_robot_get_device("vacuum gripper");
  // load and start forward motion
  WbMotionRef start_motion = wbu_motion_new("start_move.motion");
  WbMotionRef second_motion = wbu_motion_new("second_move.motion");

  // forever
  // bool reverse = false;
  move_robot(start_motion);
  wb_vacuum_gripper_turn_on(vacuum_gripper);
  delay(1);
  move_robot(second_motion);
  
  
  wb_robot_cleanup();
  // never reached
  return 0;
}
