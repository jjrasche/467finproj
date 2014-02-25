
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"

lcm_t* lcm;

static void
motor_feedback_handler(const lcm_recv_buf_t *rbuf, const char* channel, const maebot_motor_feedback_t* msg, void* user)
{
    printf("%lld,\t%d,\t\t%d\n", msg->utime,
           msg->encoder_left_ticks,
           msg->encoder_right_ticks);
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


