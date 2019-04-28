/*
 * File:          parking.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/motor.h>

// Utils
#include <fcntl.h>


#define TIME_STEP 64
#define MAXLINE 150


// parameters from proto
#define FRONT_WHEEL_RADIUS 0.38
#define REAR_WHEEL_RADIUS 0.6
double wheelbase = 4.0;
double trackFront = 1.7;



int main(int argc, char **argv) {
  wbu_driver_init();
  double velocity = 10.0;
  double left_steer = -35.0;
  double right_steer = 35.0;
  double straight = 0.0;

  FILE *fp;
  
  char line[MAXLINE];
  if ((fp = fopen(argv[1], "r")) == NULL) {
    printf("cannot open file %s\n", argv[1] );
    return 1;
  }
  wbu_driver_set_cruising_speed(velocity);
  int keep_exec = 0;
  while (wbu_driver_step(TIME_STEP) != -1) {
    if(keep_exec == 0) {
      if (fgets(line, sizeof line, fp ) == NULL) {
        break;
      }
      char cmd;
      int distance;
      sscanf(line, "%c %d", &cmd, &distance);
      printf("COMMAND: %c\n", cmd);
      printf("DISTANCE: %d\n", distance);
      switch(cmd) {
        case 'R':
        case 'r': wbu_driver_set_steering_angle(1.0);
                  break;
        case 'L':
        case 'l': wbu_driver_set_steering_angle(-1.0);
                  break;
        case 'S':
        case 's': wbu_driver_set_steering_angle(0.0);
                  break;
      }
      keep_exec = distance;
    }
    else
      printf("KEEP EXEC\n");
    keep_exec --;
  }
  wbu_driver_set_cruising_speed(0.0);
  wbu_driver_cleanup();

  return 0;
}
