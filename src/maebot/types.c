
#include "types.h"

void serialize_state(state_t* state, void* buf)
{
    int32_t*  b32  = (int32_t*)  buf;
    int16_t*  b16  = (int16_t*)  buf;
    uint16_t* bu16 = (uint16_t*) buf;
    uint8_t*  bu8  = (uint8_t*)   buf;

    b32[0] = state->encoder_left_ticks;
    b32[1] = state->encoder_right_ticks;

    b16[4] = state->motor_left_speed_cmd;
    b16[5] = state->motor_right_speed_cmd;

    b16[6] = state->accel[0];
    b16[7] = state->accel[1];
    b16[8] = state->accel[2];
    b16[9] = state->gyro[0];
    b16[10] = state->gyro[1];
    b16[11] = state->gyro[2];

    bu16[12] = state->line_sensors[0];
    bu16[13] = state->line_sensors[1];
    bu16[14] = state->line_sensors[2];

    bu16[15] = state->range;

    bu16[16] = state->motor_current_left;
    bu16[17] = state->motor_current_right;

    bu8[36] = state->pwm_prea;
    bu8[37] = state->pwm_diva;
    bu16[19] = state->pwm_prd;

    bu8[40] = state->flags;

    return;
}

void deserialize_state(void* buf, state_t* state)
{
    int32_t*  b32  = (int32_t*)  buf;
    int16_t*  b16  = (int16_t*)  buf;
    uint16_t* bu16 = (uint16_t*) buf;
    uint8_t*  bu8  = (uint8_t*)  buf;

    state->encoder_left_ticks = b32[0];
    state->encoder_right_ticks = b32[1];

    state->motor_left_speed_cmd  = b16[4];
    state->motor_right_speed_cmd = b16[5];

    state->accel[0] = b16[6];
    state->accel[1] = b16[7];
    state->accel[2] = b16[8];
    state->gyro[0]  = b16[9];
    state->gyro[1]  = b16[10];
    state->gyro[2]  = b16[11];

    state->line_sensors[0] = bu16[12];
    state->line_sensors[1] = bu16[13];
    state->line_sensors[2] = bu16[14];

    state->range = bu16[15];

    state->motor_current_left = bu16[16];
    state->motor_current_right = bu16[17];

    state->pwm_prea = bu8[36];
    state->pwm_diva = bu8[37];
    state->pwm_prd = bu16[19];

    state->flags = bu8[40];

    return;
}

void serialize_command(command_t* command, void* buf)
{
    //uint32_t* bu32 = (uint32_t*) buf;
    uint16_t*  bu16  = (uint16_t*)  buf;
    uint8_t*  bu8  = (uint8_t*)  buf;

    bu16[0] = command->motor_left_speed;
    bu16[1] = command->motor_right_speed;

    bu8[4] = command->pwm_prea;
    bu8[5] = command->pwm_diva;
    bu16[3] = command->pwm_prd;

    bu8[8] = command->flags;

    return;
}

void deserialize_command(void* buf, command_t* command)
{
    //uint32_t* bu32 = (uint32_t*) buf;
    int16_t* bu16 = (int16_t*) buf;
    uint8_t* bu8 = (uint8_t*) buf;

    command->motor_left_speed = bu16[0];
    command->motor_right_speed = bu16[1];

    command->pwm_prea = bu8[36];
    command->pwm_diva = bu8[37];
    command->pwm_prd = bu16[19];

    command->flags = bu8[8];

    return;
}


uint8_t calc_checksum(uint8_t* buf, uint32_t len)
{
    if(len <= 0) return 0;
    uint8_t checksum = buf[0];
    uint32_t i;
    for(i = 1; i < len; i++)
    {
        checksum = checksum ^ buf[i];
    }
    return checksum;
}
