#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define PI 3.1415
#define OBSTACLE_DISTANCE .17

enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE
};

double encoder_left_1_1,encoder_left_1_dif;
double encoder_left_2_1, encoder_left_2_dif;
double encoder_right_1_1, encoder_right_1_dif;
double encoder_right_2_1, encoder_right_2_dif;
double encoder_left_1_2 = 0;
double encoder_left_2_2 = 0;
double encoder_right_1_2 = 0;
double encoder_right_2_2 = 0;
double dis_value_left;
double dis_radar;
double dis_sen_left;
double encoder_left;
double encoder_right;
double value_cm_left;
double value_m_radar;
double pos_radar, pos_gun;
double turn_l;
double turn_r;
double radar_giro;
double dis_sen;
double radio=0.075;
double initial_angle_wheel1;

float encoder_left_1_angv, encoder_left_2_angv;
float encoder_right_1_angv, encoder_right_2_angv;
float encoder_left_1_linv, encoder_left_2_linv;
float encoder_right_1_linv, encoder_right_2_linv;
float encoder_left_1_rpm, encoder_left_2_rpm;
float encoder_right_1_rpm, encoder_right_2_rpm;

void goRobot(WbDeviceTag *wheels) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 4);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], 4);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], 4);
  wb_motor_set_position(wheels[3], INFINITY);
  wb_motor_set_velocity(wheels[3], 4);
}
void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
  wb_motor_set_velocity(wheels[3], 0);
}
void turnRight(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0],  -2);
  wb_motor_set_velocity(wheels[1],   2);
  wb_motor_set_velocity(wheels[2],  -2);
  wb_motor_set_velocity(wheels[3],   2);
}
void radar_on(WbDeviceTag radar_motor) {
  WbDeviceTag wheel_left_1  = wb_robot_get_device("motor_wheel_left_1");
  WbDeviceTag wheel_left_2  = wb_robot_get_device("motor_wheel_left_2");
  WbDeviceTag wheel_right_1 = wb_robot_get_device("motor_wheel_right_1");
  WbDeviceTag wheel_right_2 = wb_robot_get_device("motor_wheel_right_2");

  WbDeviceTag wheels[4];
  wheels[0] = wheel_left_1;
  wheels[1] = wheel_right_1;
  wheels[2] = wheel_left_2;
  wheels[3] = wheel_right_2;
  WbDeviceTag distance_radar = wb_robot_get_device("distance_sensor_radar");
  wb_motor_set_position(radar_motor, INFINITY);
  wb_motor_set_velocity(radar_motor, .5);
  dis_radar = wb_distance_sensor_get_value(distance_radar);
  value_m_radar = ((dis_radar *2)/1023);
  if (value_m_radar<=1.5 ) {
     stopRobot(wheels);
     wb_motor_set_position(radar_motor, INFINITY);
     wb_motor_set_velocity(radar_motor, 0);
     radar_giro++;
     WbDeviceTag Radar_encoder = wb_robot_get_device("Radar_encoder");
     WbDeviceTag gun_motor     = wb_robot_get_device("Gun_motor");
     pos_radar=wb_position_sensor_get_value(Radar_encoder);
     wb_motor_set_velocity(radar_motor, .01);
     wb_motor_set_position(gun_motor, (pos_radar-3.1416));
     wb_motor_set_velocity(radar_motor, 0);

   }
   if (radar_giro>=1){
     stopRobot(wheels);
     wb_motor_set_position(radar_motor, INFINITY);
     wb_motor_set_velocity(radar_motor, 0);
     radar_giro++;
     if (value_m_radar<=.5) {
       printf("THA!\n");
     } else if (value_m_radar<=1) {
       printf("THA THAA!!\n");
     } else if (value_m_radar<=1.25){
       printf("THA THAA THAAA!!!\n");
     }
     if (radar_giro >= 60 && value_m_radar >= 2 )
     {
       radar_giro = 0;
       wb_motor_set_position(radar_motor, INFINITY);
       wb_motor_set_velocity(radar_motor, .5);
     }
   }
}
void checkForObstacles(WbDeviceTag distance_sensor_left){
  WbDeviceTag wheel_left_1  = wb_robot_get_device("motor_wheel_left_1");
  WbDeviceTag wheel_left_2  = wb_robot_get_device("motor_wheel_left_2");
  WbDeviceTag wheel_right_1 = wb_robot_get_device("motor_wheel_right_1");
  WbDeviceTag wheel_right_2 = wb_robot_get_device("motor_wheel_right_2");

  WbDeviceTag wheels[4];
  wheels[0] = wheel_left_1;
  wheels[1] = wheel_right_1;
  wheels[2] = wheel_left_2;
  wheels[3] = wheel_right_2;
  dis_value_left = wb_distance_sensor_get_value(distance_sensor_left);
  value_cm_left = ((dis_value_left *.4)/255);
  if (value_cm_left<=0.2 ) {
     turn_l++;
   }
   if (turn_l>=1 && turn_l<=50){
     turnRight(wheels);
     turn_l++;
   }

   else {
     turn_l=0;
   }
}

int main(int argc, char **argv)
{

wb_robot_init();
WbDeviceTag wheel_left_1  = wb_robot_get_device("motor_wheel_left_1");
WbDeviceTag wheel_left_2  = wb_robot_get_device("motor_wheel_left_2");
WbDeviceTag wheel_right_1 = wb_robot_get_device("motor_wheel_right_1");
WbDeviceTag wheel_right_2 = wb_robot_get_device("motor_wheel_right_2");
WbDeviceTag radar_motor   = wb_robot_get_device("Radar_motor");
WbDeviceTag gun_motor     = wb_robot_get_device("Gun_motor");

WbDeviceTag wheels[4];
wheels[0] = wheel_left_1;
wheels[1] = wheel_right_1;
wheels[2] = wheel_left_2;
wheels[3] = wheel_right_2;

WbDeviceTag encoder_left_1  = wb_robot_get_device("encoder_wheel_left_1");
WbDeviceTag encoder_left_2  = wb_robot_get_device("encoder_wheel_left_2");
WbDeviceTag encoder_right_1 = wb_robot_get_device("encoder_wheel_right_1");
WbDeviceTag encoder_right_2 = wb_robot_get_device("encoder_wheel_right_2");
WbDeviceTag Radar_encoder   = wb_robot_get_device("Radar_encoder");
WbDeviceTag Gun_encoder     = wb_robot_get_device("Gun_encoder");

WbDeviceTag distance_sensor_left  = wb_robot_get_device("distance_sensor_left");
WbDeviceTag distance_radar = wb_robot_get_device("distance_sensor_radar");
/*WbDeviceTag distance_sensor_gun   = wb_robot_get_device("gun_distance_sensor");*/

wb_position_sensor_enable(encoder_left_1, TIME_STEP);
wb_position_sensor_enable(encoder_right_1, TIME_STEP);
wb_position_sensor_enable(encoder_left_2, TIME_STEP);
wb_position_sensor_enable(encoder_right_2, TIME_STEP);
wb_position_sensor_enable(Gun_encoder, TIME_STEP);
wb_position_sensor_enable(Radar_encoder, TIME_STEP);
wb_distance_sensor_enable(distance_sensor_left, TIME_STEP);
wb_distance_sensor_enable(distance_radar, TIME_STEP);


while (wb_robot_step(TIME_STEP) != -1) {
  goRobot(wheels);
  radar_on(radar_motor);
  checkForObstacles(distance_sensor_left);




}

wb_robot_cleanup();

return 0;
}
