
#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include "common/timestamp.h"
#include "lcmtypes/maebot_diff_drive_t.h"

#define CMD_PRD 50000 //us  -> 20Hz

lcm_t* lcm;
maebot_diff_drive_t msg;
pthread_mutex_t msg_mutex;

void* diff_drive_thread(void* arg)
{
    uint64_t utime_start;

    while(1)
    {
        utime_start = utime_now();

        pthread_mutex_lock(&msg_mutex);
        maebot_diff_drive_t_publish(lcm, "MAEBOT_DIFF_DRIVE", &msg);
        pthread_mutex_unlock(&msg_mutex);

        usleep(CMD_PRD - (utime_now() - utime_start));
    }

    return NULL;
}

int main()
{
    if(pthread_mutex_init(&msg_mutex, NULL))
    {
        printf("mutex init failed\n");
        return 1;
    }

    // Init msg
    // no need for mutex here, as command thread hasn't started yet.
    msg.motor_left_speed = 0.0f;
    msg.motor_right_speed = 0.0f;

    // Start sending motor commands
    pthread_t diff_drive_thread_pid;
    pthread_create(&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

    // forward
    pthread_mutex_lock(&msg_mutex);
    msg.motor_left_speed  = 0.2f;
    msg.motor_right_speed = 0.2f;
    pthread_mutex_unlock(&msg_mutex);

    usleep(250000);

    // reverse
    pthread_mutex_lock(&msg_mutex);
    msg.motor_left_speed  = -0.2f;
    msg.motor_right_speed = -0.2f;
    pthread_mutex_unlock(&msg_mutex);

    usleep(250000);

    // left turn
    pthread_mutex_lock(&msg_mutex);
    msg.motor_left_speed  = -0.2f;
    msg.motor_right_speed = 0.2f;
    pthread_mutex_unlock(&msg_mutex);

    usleep(250000);

    // right turn
    pthread_mutex_lock(&msg_mutex);
    msg.motor_left_speed  = 0.2f;
    msg.motor_right_speed = -0.2f;
    pthread_mutex_unlock(&msg_mutex);

    usleep(250000);

    // stop
    pthread_mutex_lock(&msg_mutex);
    msg.motor_left_speed  = 0.0f;
    msg.motor_right_speed = 0.0f;
    pthread_mutex_unlock(&msg_mutex);

    return 0;
}
