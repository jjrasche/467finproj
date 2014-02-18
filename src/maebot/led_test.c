
#include <stdint.h>
#include <lcm/lcm.h>
#include <lcmtypes/maebot_leds_t.h>

int main()
{
    lcm_t* lcm = lcm_create(NULL);
    maebot_leds_t msg;

    msg.bottom_led_left = true;
    msg.bottom_led_middle = true;
    msg.bottom_led_right = true;
    msg.line_sensor_leds = true;

    // Red
    msg.top_rgb_led_left = 0x100000;
    msg.top_rgb_led_right = 0x100000;

    maebot_leds_t_publish(lcm, msg, "MAEBOT_LEDS");

    usleep(200000);

    // Green
    msg.top_rgb_led_left = 0x1000;
    msg.top_rgb_led_right = 0x1000;

    maebot_leds_t_publish(lcm, msg, "MAEBOT_LEDS");

    usleep(200000);

    // Blue
    msg.top_rgb_led_left = 0x10;
    msg.top_rgb_led_right = 0x10;

    maebot_leds_t_publish(lcm, msg, "MAEBOT_LEDS");

    usleep(200000);

    // White
    msg.top_rgb_led_left = 0x101010;
    msg.top_rgb_led_right = 0x101010;

    maebot_leds_t_publish(lcm, msg, "MAEBOT_LEDS");

    usleep(200000);


    msg.bottom_led_left = false;
    msg.bottom_led_middle = false;
    msg.bottom_led_right = false;
    msg.line_sensor_leds = true;

    // Off
    msg.top_rgb_led_left = 0x0;
    msg.top_rgb_led_right = 0x0;

    maebot_leds_t_publish(lcm, msg, "MAEBOT_LEDS");


    return 0;
}
