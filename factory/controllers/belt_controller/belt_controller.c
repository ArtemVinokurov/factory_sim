/*
 * File:          belt_controller.c
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
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/motor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 8


#define BELT_SPEED 0.1

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  WbDeviceTag belt_motor = wb_robot_get_device("belt_motor");
  WbDeviceTag ps = wb_robot_get_device("box_trigger");
  
  wb_motor_set_position(belt_motor, INFINITY);
  wb_motor_set_velocity(belt_motor, BELT_SPEED);
  printf("Hello");
  wb_distance_sensor_enable(ps, TIME_STEP);
  double distance = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    distance = wb_distance_sensor_get_value(ps);
    printf("sensor: %f", distance);
    if (distance < 50 && distance > 40)
    {
      wb_motor_set_velocity(belt_motor, 0.0);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
