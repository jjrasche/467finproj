
#include <lcmtypes/maebot_laser_t.h>
#include <unistd.h>

#define NUM_BLINKS 3

int main()
{
  lcm_t* lcm = lcm_create(NULL);
  maebot_laser_t msg;

  msg.laser_power = 1;
  maebot_laser_t_publish(lcm, "MAEBOT_LASER", &msg);
  usleep(250000);

  int i;
  for(i = 1; i < NUM_BLINKS; i++)
    {
      msg.laser_power = 0;
      maebot_laser_t_publish(lcm, "MAEBOT_LASER", &msg);
      usleep(250000);
      
      msg.laser_power = 1;
      maebot_laser_t_publish(lcm, "MAEBOT_LASER", &msg);
      usleep(250000);
    }

  msg.laser_power = 0;
  maebot_laser_t_publish(lcm, "MAEBOT_LASER", &msg);

  return 0;
}