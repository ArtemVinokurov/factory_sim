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
int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  // load and start forward motion
  WbMotionRef motion = wbu_motion_new("puma560.motion");
  wbu_motion_play(motion);

  // forever
  // bool reverse = false;
  // while (1) {
    // reverse = reverse ? false : true;
    // wbu_motion_set_reverse(motion, reverse);

    while (!wbu_motion_is_over(motion))
      wb_robot_step(time_step);

  // never reached
  return 0;
}

