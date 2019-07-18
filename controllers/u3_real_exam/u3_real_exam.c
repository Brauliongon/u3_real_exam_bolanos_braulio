#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64
#define PI 3.1415
#define OBSTACLE_DISTANCE 40.0

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
double dis_sen_left;
double encoder_left;
double encoder_right;
double value_cm_left;
double value_cm_right;
double pos_radar, pos_gun;
double turn_l;
double turn_r;
double giro;
double dis_sen;
double radio=0.075;
double initial_angle_wheel1;

float encoder_left_1_angv, encoder_left_2_angv;
float encoder_right_1_angv, encoder_right_2_angv;
float encoder_left_1_linv, encoder_left_2_linv;
float encoder_right_1_linv, encoder_right_2_linv;
float encoder_left_1_rpm, encoder_left_2_rpm;
float encoder_right_1_rpm, encoder_right_2_rpm;

double ps_value;
short int ds_state, robot_state = GO;
float angle;


int checkForObstacles(WbDeviceTag distance_sensor_left) {
  double distance = wb_distance_sensor_get_value(distance_sensor_left);

  if (distance > OBSTACLE_DISTANCE)
    return FREEWAY;
  else
    return OBSTACLE;
}

void goRobot(WbDeviceTag *wheels) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 2);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], 2);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], 2);
  wb_motor_set_position(wheels[3], INFINITY);
  wb_motor_set_velocity(wheels[3], 2);
}

void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
  wb_motor_set_velocity(wheels[3], 0);
}

void turnRight(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -0.3);
  wb_motor_set_velocity(wheels[1], -0.3);
  wb_motor_set_velocity(wheels[2], 0.3);
  wb_motor_set_velocity(wheels[3], 0.3);
}

/*
double getAngleRobot(WbDeviceTag pos_sensor) {
  printf("Calculando angulo\n");
  double angle, angle_wheel1, angle_wheel2;

  angle_wheel1 = wb_position_sensor_get_value(pos_sensor);
  printf("Angle Wheel1: %lf\n", angle_wheel1);
  angle = fabs(angle_wheel1 - initial_angle_wheel1);
  printf("Angle: %lf\n", angle);

  return angle;
}

float clearAngleRobot() {
  printf("Clearing angle\n");
}
*/
int main(int argc, char **argv)
{

wb_robot_init();

WbDeviceTag wheel_left_1  = wb_robot_get_device("motor_wheel_left_1");
WbDeviceTag wheel_left_2  = wb_robot_get_device("motor_wheel_left_2");
WbDeviceTag wheel_right_1 = wb_robot_get_device("motor_wheel_right_1");
WbDeviceTag wheel_right_2 = wb_robot_get_device("motor_wheel_right_2");
WbDeviceTag radar_motor   = wb_robot_get_device("Radar_motor");
WbDeviceTag gun_motor     = wb_robot_get_device("Gun_encoder");

WbDeviceTag wheels[4];
wheels[0] = wheel_left_1;
wheels[1] = wheel_right_1;
wheels[2] = wheel_left_2;
wheels[3] = wheel_right_2;

WbDeviceTag top[4];
top[0] = radar_motor;
top[1] = gun_motor;

WbDeviceTag encoder_left_1  = wb_robot_get_device("encoder_wheel_left_1");
WbDeviceTag encoder_left_2  = wb_robot_get_device("encoder_wheel_left_2");
WbDeviceTag encoder_right_1 = wb_robot_get_device("encoder_wheel_right_1");
WbDeviceTag encoder_right_2 = wb_robot_get_device("encoder_wheel_right_2");
WbDeviceTag encoder_radar   = wb_robot_get_device("Radar_encoder");
WbDeviceTag encoder_gun     = wb_robot_get_device("Gun_encoder");

WbDeviceTag distance_sensor_left  = wb_robot_get_device("distance_sensor_left");
WbDeviceTag distance_sensor_radar = wb_robot_get_device("radar_distance_sensor");
WbDeviceTag distance_sensor_gun   = wb_robot_get_device("gun_distance_sensor");

wb_position_sensor_enable(encoder_left_1, TIME_STEP);
wb_position_sensor_enable(encoder_right_1, TIME_STEP);
wb_position_sensor_enable(encoder_left_2, TIME_STEP);
wb_position_sensor_enable(encoder_right_2, TIME_STEP);
wb_position_sensor_enable(encoder_gun, TIME_STEP);
wb_position_sensor_enable(encoder_radar, TIME_STEP);
wb_distance_sensor_enable(distance_sensor_left, TIME_STEP);

while (wb_robot_step(TIME_STEP) != -1) {
  if (robot_state == GO) {
      ds_state = checkForObstacles(distance_sensor_left);

      if (ds_state == FREEWAY) {
        goRobot(wheels);

      }
    }

/*
wb_position_sensor_get_value(encoder_left_1);
wb_position_sensor_get_value(encoder_left_2);
wb_position_sensor_get_value(encoder_right_1);
wb_position_sensor_get_value(encoder_right_2);
*/
/*encoder_left_1_1 = wb_position_sensor_get_value(encoder_left_1);
encoder_left_1_dif = encoder_left_1_1 - encoder_left_1_2;
encoder_left_1_2 = encoder_left_1_1;
encoder_left_1_angv = encoder_left_1_dif/0.064;
encoder_left_1_linv = encoder_left_1_angv*radio;
encoder_left_1_rpm = -1*encoder_left_1_angv*(60/(2*PI));

encoder_left_2_1 = wb_position_sensor_get_value(encoder_left_2);
encoder_left_2_dif = encoder_left_2_1 - encoder_left_2_2;
encoder_left_2_2 = encoder_left_2_1;
encoder_left_2_angv = encoder_left_2_dif/0.064;
encoder_left_2_linv = encoder_left_2_angv*radio;
encoder_left_2_rpm = -1*encoder_left_2_angv*(60/(2*PI));

encoder_right_1_1 = wb_position_sensor_get_value(encoder_right_1);
encoder_right_1_dif = encoder_right_1_1 - encoder_right_1_2;
encoder_right_1_2 = encoder_right_1_1;
encoder_right_1_angv = encoder_right_1_dif/0.064;
encoder_right_1_linv = encoder_right_1_angv*radio;
encoder_right_1_rpm = -1*(encoder_right_1_angv*(60/(2*PI)));

encoder_right_2_1 = wb_position_sensor_get_value(encoder_right_2);
encoder_right_2_dif = encoder_right_2_1 - encoder_right_2_2;
encoder_right_2_2 = encoder_right_2_1;
encoder_right_2_angv = encoder_right_2_dif/0.064;
encoder_right_2_linv = encoder_right_2_angv*radio;
encoder_right_2_rpm = -1*(encoder_right_2_angv*(60/(2*PI)));*/

/*
dis_value_left = wb_distance_sensor_get_value(distance_sensor_left);
value_cm_left = ((dis_value_left *.2)/65535);
pos_gun = wb_position_sensor_get_value(encoder_gun);
pos_radar = wb_position_sensor_get_value(encoder_radar);
printf("radar%2f\n",pos_radar );
printf("gun%2f\n",pos_gun );
printf("Distancia left: %.2f  \n", value_cm_left);
*/
/////////////////////motor y giro//////////////////

/*wb_motor_set_position(wheel_left_1, INFINITY);
wb_motor_set_velocity(wheel_left_1, 1);
wb_motor_set_position(wheel_left_2, INFINITY);
wb_motor_set_velocity(wheel_left_2, 1);
wb_motor_set_position(wheel_right_1, INFINITY);
wb_motor_set_velocity(wheel_right_1, 1);
wb_motor_set_position(wheel_right_2, INFINITY);
wb_motor_set_velocity(wheel_right_2, 1);
wb_motor_set_position(radar_motor, INFINITY);
wb_motor_set_velocity(radar_motor, 1);




if (value_cm_left<=0.17 ) {
   turn_l++;
 }
 if (turn_l>=1 && turn_l<=55){
   wb_motor_set_position(wheel_left_1, INFINITY);
   wb_motor_set_velocity(wheel_left_1, 0);
   wb_motor_set_position(wheel_left_2, INFINITY);
   wb_motor_set_velocity(wheel_left_2, 0);
   wb_motor_set_position(wheel_right_1, INFINITY);
   wb_motor_set_velocity(wheel_right_1, 1);
   wb_motor_set_position(wheel_right_2, INFINITY);
   wb_motor_set_velocity(wheel_right_2, 1);
   turn_l++;
 }

 else {
   turn_l=0;
 }

*/

}

wb_robot_cleanup();

return 0;
}
