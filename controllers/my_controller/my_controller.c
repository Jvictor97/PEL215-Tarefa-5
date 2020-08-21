#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 64

// Definicao da funcao delay
void delay (int time_milisec) {
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft = 0.00;
  while (Timeleft < timeValue)
  {
    currentTime = wb_robot_get_time();
    Timeleft = currentTime - initTime;
    wb_robot_step(TIME_STEP);
  }
}


// Funcao auxiliar para converter graus em radianos
double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double validInterval(double value, double min, double max) {
  if (value < min) return min;
  if (value > max) return max;
  
  return value;
}

void inverseKinematics(double x3, double y3, double z3, double* theta) {
  double L1 = 0.5, L2 = 2, L3 = 2;
  
  double r1 = sqrt(pow(x3,2) + pow(y3,2));
  double r2 = z3 - L1;
  double r3 = sqrt(pow(r1,2) + pow(r2,2));
  
  printf("r1: %f, r2: %f, r3: %f\n", r1, r2, r3);
  printf("cosPhi1: %f\n", (pow(L3,2) - pow(L2,2) - pow(r3,2)) / (-2*L2*r3));
  
  double cosPhi1 = (pow(L3,2) - pow(L2,2) - pow(r3,2)) / (-2*L2*r3);
  double theta1 = atan(y3/x3);
  double theta2 = atan(r2/r1) - acos(validInterval(cosPhi1, -1, 1));
   
  printf("cosPhi3: %f\n", (pow(r3,2) - pow(L2,2) - pow(L3,2)) / (-2*L2*L3));
  
  double cosPhi3 = (pow(r3,2) - pow(L2,2) - pow(L3,2)) / (-2*L2*L3);
  double theta3 = toRadians(180) - acos(validInterval(cosPhi3, -1, 1));
  printf("Theta1: %f\n", theta1);
  printf("Theta2: %f\n", theta2);
  printf("Theta3: %f\n", theta3);
  theta[0] = theta1;
  theta[1] = theta2;
  theta[2] = theta3;
}

double toPositiveAngle(double radians) {
  return radians < 0 ? radians + 2*M_PI : radians;
}

double toNegativeAngle(double radians) {
  return radians > 0 ? radians - 2*M_PI : radians;
}

int main(int argc, char **argv) {
  
  wb_robot_init();
  
  WbDeviceTag motors[3];
  motors[0] = wb_robot_get_device("base");
  motors[1] = wb_robot_get_device("upperarm");
  motors[2] = wb_robot_get_device("forearm");
  
  for (int i = 0; i < 3; ++i)
    wb_motor_set_velocity(motors[i], 1.0);
    
  double positions[][3] = {
    {0.001, 0.3, 0.5}, 
    {0.3, 0.0, 0.3},
    {0.001, -0.2, 0.5}
  };
  
  double theta[3] = { 0.0, 0.0, 0.0 };
  
  for(int i = 0; i < 3; i++) {
    double* position = positions[i];
  
    inverseKinematics(position[0], position[1], position[2], theta);
    
    double theta1 = validInterval(toPositiveAngle(theta[0]), 0, 6.03);
    double theta2 = validInterval(toNegativeAngle(theta[1]), -2.44, 0);
    double theta3 = validInterval(toPositiveAngle(theta[2]), 0, 4.21);
    
    printf("Theta1: %.5f, Theta2: %.5f, Theta3: %.5f\n", 
          theta1, theta2, theta3); 
                    
    wb_motor_set_position(motors[0], theta1);
    wb_motor_set_position(motors[1], theta2);
    wb_motor_set_position(motors[2], theta3);
    
    delay(5000);
  }
  
  wb_motor_set_position(motors[0], 0);
  wb_motor_set_position(motors[1], 0);
  wb_motor_set_position(motors[2], 0);

  wb_robot_cleanup();

  return 0;
}
