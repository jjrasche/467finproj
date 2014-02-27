
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"

lcm_t* lcm;

static void
motor_feedback_handler(const lcm_recv_buf_t *rbuf, const char* channel, const maebot_motor_feedback_t* msg, void* user)
{
    system("clear");
    printf("utime: %lld\n", msg->utime);
    printf("Subscribed to channed: MAEBOT_MOTOR_FEEDBACK");
    printf("encoder_[left, right]_ticks: %d,\t%d\n", msg->encoder_left_ticks, msg->encoder_right_ticks);
    printf("motor_current[left, right]: %d,\t%d\n", msg->motor_current_left, msg->motor_current_right);
    printf("motor_[left, right]_commanded_speed: %f,\t%f\n", msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    printf("motor_[left, right]_actual_speed: %f,\t%f\n", msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
}

int main()
{
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
	if(!lcm)
		return 1;

    printf("utime,\t\tleft_ticks,\tright_ticks\n");

    maebot_motor_feedback_t_subscribe(lcm,
                                      "MAEBOT_MOTOR_FEEDBACK",
                                      motor_feedback_handler,
                                      NULL);

    while(1)
    {
        lcm_handle(lcm);
    }

    return 0;
}
