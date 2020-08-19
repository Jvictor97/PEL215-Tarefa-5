#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 64

// Funcao auxiliar para converter graus em radianos
double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

int main(int argc, char **argv) {
  
  wb_robot_init();
  
  WbDeviceTag motors[5];
  ur_motors[0] = wb_robot_get_device("base");
  // ur_motors[1] = wb_robot_get_device("shoulder_lift_joint");
  // ur_motors[2] = wb_robot_get_device("elbow_joint");
  // ur_motors[3] = wb_robot_get_device("wrist_1_joint");
  // ur_motors[4] = wb_robot_get_device("wrist_2_joint");
  
  wb_motor_set_position(ur_motors[0], toRadians(90));


  wb_robot_cleanup();

  return 0;
}
