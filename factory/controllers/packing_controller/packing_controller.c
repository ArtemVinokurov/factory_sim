/*
 * File:          packing_controller.c
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
#include <webots/utils/motion.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  WbMotionRef motion = wbu_motion_new("move_start.motion");

  wbu_motion_play(motion);

  while (1) {
    while (!wbu_motion_is_over(motion))
      wb_robot_step(time_step);
    break;
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
