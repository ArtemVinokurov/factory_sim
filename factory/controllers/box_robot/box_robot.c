#include <webots/robot.h>
#include <webots/utils/motion.h>

int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  // load and start forward motion
  WbMotionRef motion = wbu_motion_new("puma560.motion");
  wbu_motion_play(motion);

  // forever
  // bool reverse = false;
  while (1) {
    // reverse = reverse ? false : true;
    // wbu_motion_set_reverse(motion, reverse);
    while (!wbu_motion_is_over(motion))
      wb_robot_step(time_step);
    break;
     
  }
  wb_robot_cleanup();
  // never reached
  return 0;
}
