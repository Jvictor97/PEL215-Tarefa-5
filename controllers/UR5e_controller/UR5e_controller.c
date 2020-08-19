// Inclusao das bibliotecas
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#define TIME_STEP 32

// Funcao para realizar a multiplicacao de duas matrizes 4x4
void multiplyMatrices(double firstMatrix[][4], double secondMatrix[][4], double mult[][4])
{
  int i, j, k;
  
  for(j = 0; j < 4; ++j) {
    for(k = 0; k < 4; ++k) {
      mult[k][j] = 0.0;
    }
  }
  
  for(i = 0; i < 4; ++i) {
    for(j = 0; j < 4; ++j) {
      for(k = 0; k < 4; ++k) {
        mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
      }
    }
  }
}

// Funcao auxiliar para converter graus em radianos
double toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

// Funcao para construir as matrizes de DH
void buildMatrix(double matrix[][4], double theta, double alfa, double r, double d) {
  double H[4][4] = {
    { cos(theta), -sin(theta)*cos(alfa),  sin(theta)*sin(alfa), r*cos(theta) },
    { sin(theta),  cos(theta)*cos(alfa), -cos(theta)*sin(alfa), r*sin(theta) },
    {        0.0,             sin(alfa),             cos(alfa),            d },
    {        0.0,                   0.0,                   0.0,          1.0 }
  };
  
  for (int i = 0; i < 4; i++) {
    memcpy(matrix[i], H[i], sizeof(matrix[i]));
  }
}

// Funcao para exibir no terminal uma matriz 4x4
void printMatrix(double matrix[][4]) {
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%.2f ", matrix[i][j]);
    }
    printf("\n");
  }
  
  printf("\n");
}

// Funcao para estimar a posicao final do robo pelo metodo DH
void denavitHartenberg(double* theta) {
  // Declaracao das variaveis para as matrizes
  double H1[4][4];
  double H2[4][4];
  double H3[4][4];
  double H4[4][4];
  double H5[4][4];
  double H6[4][4]; 
  
  // Construcao das matrizes de DH utilizando os valores de Theta recebidos
  //                            theta            alfa        r        d  
  buildMatrix(H1, toRadians(theta[0]),  toRadians(90),       0,  0.1625	);  
  buildMatrix(H2, toRadians(theta[1]),            0.0,  -0.425,     0.0);
  buildMatrix(H3, toRadians(theta[2]),            0.0, -0.3922,     0.0);
  buildMatrix(H4, toRadians(theta[3]),  toRadians(90),     0.0,  0.1333);
  buildMatrix(H5, toRadians(theta[4]), toRadians(-90),     0.0,  0.0997);
  buildMatrix(H6, toRadians(theta[5]),            0.0,     0.0,  0.0996);
  
  // Matriz auxiliar para uso nos calculos
  double firstTempMatrix[4][4]  = { 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0} 
  };
  
  // Matriz auxiliar para uso nos calculos
  double secondTempMatrix[4][4]  = { 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0}, 
    {0.0,0.0,0.0,0.0} 
  };
  
  // Multiplicacao das matrizes de DH
  multiplyMatrices(H1, H2, firstTempMatrix);  
  multiplyMatrices(firstTempMatrix, H3, secondTempMatrix);
  multiplyMatrices(secondTempMatrix, H4, firstTempMatrix);
  multiplyMatrices(firstTempMatrix, H5, secondTempMatrix);
  multiplyMatrices(secondTempMatrix, H6, firstTempMatrix);
  
  // Vetor para a posicao inicial do robo
  double initialPosition[4] = { 0.0, 0.0, 0.0, 1.0 };
  // Vetor para a posicao final do robo
  double finalPosition[4]   = { 0.0, 0.0, 0.0, 1.0 };
  
  // Multiplicacao da matriz final de DH pela posicao inicial
  // para obtencao da posicao final estimada
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      finalPosition[i] += firstTempMatrix[i][j] * initialPosition[j];
    }
  }

  // Printa no terminal a posicao estimada
  printf("Posicao Final - X: %f; Y: %f; Z: %f\n", 
  finalPosition[0], finalPosition[1], finalPosition[2]);
}

// Funcao principal
int main(int argc, char **argv) {
  wb_robot_init();
  int i = 0;
 
  // Angulos de rotacao aplicados nos eixos do robo
  double target_positions[] = {90, -90, -45, 0, 0, 0};
  double speed = 1.0;

  // Obtendo de acesso aos motores do robo
  WbDeviceTag ur_motors[6];
  ur_motors[0] = wb_robot_get_device("shoulder_pan_joint");
  ur_motors[1] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[2] = wb_robot_get_device("elbow_joint");
  ur_motors[3] = wb_robot_get_device("wrist_1_joint");
  ur_motors[4] = wb_robot_get_device("wrist_2_joint");
  ur_motors[5] = wb_robot_get_device("wrist_3_joint");
  
  // Configura a velocidade dos motores
  for (i = 0; i < 6; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  // Estima a posicao final do robo com base na inicial
  // e nos angulos de movimentacao
  denavitHartenberg(target_positions);
  
  // Executa a movimentacao no robo manipulador
  for (i = 0; i < 6; ++i) {
    wb_motor_set_position(ur_motors[i], toRadians(target_positions[i]));
  }

  // Limpeza de ambiente do Webots
  wb_robot_cleanup();
  return 0;
}
