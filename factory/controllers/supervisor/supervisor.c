/*
 * File:          supervisor.c
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
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 8
#define BELT_SPEED 0.25

enum {WAIT_BOX, MOVE_BOX, WAIT_STICK};   

int main(int argc, char **argv) {
  wb_robot_init();
  WbNodeRef belt_node = wb_supervisor_node_get_from_def("conveyor_belt");
  WbFieldRef belt_speed = wb_supervisor_node_get_field(belt_node, "speed");
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("stick_robot");
  WbFieldRef robot_state = wb_supervisor_node_get_field(robot_node, "customData");
  
  WbDeviceTag box_detector_1 = wb_robot_get_device("box_trigger_1");
  WbDeviceTag box_detector_2 = wb_robot_get_device("box_trigger_2");
  WbDeviceTag nxt_box_trigger = wb_robot_get_device("next_box_trigger");
  
  double distance = 0;
  wb_distance_sensor_enable(box_detector_1, TIME_STEP);
  wb_distance_sensor_enable(box_detector_2, TIME_STEP);
  wb_distance_sensor_enable(nxt_box_trigger, TIME_STEP);
  
  
  wb_supervisor_field_set_sf_float(belt_speed, BELT_SPEED);
  int state = WAIT_BOX;
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, "CardboardBox {translation 1.7 -1.026 0.54 size 0.25 0.25 0.25 mass 0.1}");
  while (wb_robot_step(TIME_STEP) != -1) {
      switch(state)
      {
        case WAIT_BOX:
          if (wb_distance_sensor_get_value(nxt_box_trigger) < 400)
          {
            state = MOVE_BOX;
          }
         
          break;
          
        case MOVE_BOX:
          if (wb_distance_sensor_get_value(box_detector_1) < 400 && wb_distance_sensor_get_value(box_detector_2) < 400)
          {
            wb_supervisor_field_set_sf_float(belt_speed, 0.0);
            wb_supervisor_field_set_sf_string(robot_state, "start");
            state = WAIT_STICK;
            wb_robot_step(TIME_STEP);
          }
          break;
          
        case WAIT_STICK:
          if (strcmp(wb_supervisor_field_get_sf_string(robot_state), "ready") == 0)
          {
            wb_supervisor_field_set_sf_float(belt_speed, BELT_SPEED);
            state = WAIT_BOX;
            wb_supervisor_field_import_mf_node_from_string(children_field, -1, "CardboardBox {translation 1.7 -1.026 0.54 size 0.25 0.25 0.25 mass 0.1}");
          }
          break;
      }
      //printf("state: %i\n", state); 
  };

  wb_robot_cleanup();

  return 0;
}
