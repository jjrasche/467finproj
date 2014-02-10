
#include <stdint.h>
#include <lcm/lcm.h>
#include "maebot_command_t.h"
#include "maebot_state_t.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>

#include <pthread.h>

#include "types.h"

#include <sys/time.h>

const uint32_t HEADER_BYTES = 12;
const uint32_t UART_MAGIC_NUMBER = 0xFDFDFDFD;  // Marks Beginning of Message

lcm_t* lcm;
int port;

static void
command_handler(const lcm_recv_buf_t *rbuf, const char* channel,
	const maebot_command_t* msg, void* user)
{
	printf("Received message on channel \"%s\":\r\n", channel);
	//printf("  timestamp   = %"PRId64"\n", msg->timestamp);
	printf("  left_speed  = %d \r\n", msg->motor_left_speed);
        printf("  right_speed = %d \r\n", msg->motor_right_speed);

	// Pass through to sama5
	command_t command;
	const uint32_t msg_sz = HEADER_BYTES + COMMAND_T_BUFFER_BYTES + 1;
	uint8_t buf[msg_sz];

	command.motor_left_speed = msg->motor_left_speed;
	command.motor_right_speed = msg->motor_right_speed;

	command.flags = 0;

	((uint32_t*)buf)[0] = UART_MAGIC_NUMBER;
	((uint32_t*)buf)[1] = COMMAND_T_BUFFER_BYTES;//msg_sz;
	((uint32_t*)buf)[2] = COMMAND_TYPE;

	serialize_command(&command, (void*)(buf + HEADER_BYTES));

	int i;
        printf("buf: ");
        for(i = 0; i < COMMAND_T_BUFFER_BYTES; i++)
        {
		printf("%d ", (int) (((uint8_t*)buf + HEADER_BYTES)[i]));
        }

	buf[msg_sz - 1] = calc_checksum(buf + HEADER_BYTES, COMMAND_T_BUFFER_BYTES);

	writen(port, buf, msg_sz);	
}


int open_port()
{
    //attempt to open port
    int fd;

    if((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
    {
	    printf("error opening /dev/ttyO1\r\n");
    }
    else
    {
	    fcntl(fd, F_SETFL, 0);
    }

    return fd;
}

int configure_port(int fd)
{
    struct termios old_settings, new_settings;
    if(tcgetattr(fd, &old_settings) != 0)
    {
	    printf("error reading port config\r\n");
	    return -1;
    }
    
    cfmakeraw(&old_settings);
    cfsetspeed(&old_settings, B115200);

    if(tcsetattr(fd, TCSANOW, &old_settings))
    {
	    printf("error setting port config\r\n");
	    return -1;
    }
    return fd;
}

int readn(int fd, void *buf, unsigned int count)
{
	int ret;
	int sofar = 0;
	while(sofar < count)
	{
		ret = read(fd, ((char*)buf) + sofar, count - sofar);
		if(ret <= 0)
		{
			printf("Error or end of file. %d\n", ret);
			return 1;
		}
		sofar += ret;
	}
	return sofar;
}

int writen(int fd, const void* buf, size_t count)
{
	int ret;
	int i = 0;
	while(i < count)
	{
		ret = write(fd, ((char*)buf) + i, count - i);
		if(ret <= 0)
		{
			printf("Error writing to file descriptor.\n");
			return 1;
		}
		i += ret;
	}
	return i;
}

state_t get_state(int port)
{
	state_t state;
	int ret;	

	uint8_t magic_check;
	uint8_t num_magic;

	uint8_t done = 0;
	while(!done)
	{

	magic_check = 0;
	num_magic = 0;

	while(num_magic != 4)
	{
		readn(port, (void*)&magic_check, 1);
		if(magic_check == 0xFD) num_magic++;
	}

	uint32_t size = 0;
        ret = readn(port, (void*)&size, 4);

        //printf("Message size: %d\n", size);

        uint32_t type = 0;
        ret = readn(port, (void*)&type, 4);

	if(type == STATE_TYPE)
	{
       		if(size != STATE_T_BUFFER_BYTES)
		{
			printf("Bad packet: expected size=%d, found size=%d\r\n"
					,STATE_T_BUFFER_BYTES, size);
			continue;
		}
	        uint8_t buf[STATE_T_BUFFER_BYTES];
	        ret = readn(port, buf, STATE_T_BUFFER_BYTES);

		uint8_t checksum;
		ret = readn(port, (void*)&checksum, 1);

		if(checksum != calc_checksum(buf, STATE_T_BUFFER_BYTES))
           	{
                	printf("Bad packet: checksum failed\r\n");
                	continue;
            	}
	
	        deserialize_state(buf, &state);

		done = 1;
	}
	else
	{
		printf("Unrecognized type: %d\r\n", type);
		continue;
	}
	}

	return state;
}


void* encoder_thread(void* arg)
{
	state_t state;
	maebot_state_t lcm_state;
	struct timeval tv;
	struct timezone tz;

	printf("Encoder Thread\n");
	while(1){	
		// Telemetry handling
		state = get_state(port);
		gettimeofday(&tv, &tz);
		lcm_state.timestamp = tv.tv_usec * 1000;
		lcm_state.encoder_left_ticks = state.encoder_left_ticks;
		lcm_state.encoder_right_ticks = state.encoder_right_ticks;
		lcm_state.motor_left_speed_cmd = state.motor_left_speed_cmd;
		lcm_state.motor_right_speed_cmd = state.motor_right_speed_cmd;
		lcm_state.accel[0] = state.accel[0];
		lcm_state.accel[1] = state.accel[1];
		lcm_state.accel[2] = state.accel[2];
		lcm_state.gyro[0] = state.gyro[0];
		lcm_state.gyro[1] = state.gyro[1];
		lcm_state.gyro[2] = state.gyro[2];
		lcm_state.line_sensors[0] = state.line_sensors[0];
		lcm_state.line_sensors[1] = state.line_sensors[1];
		lcm_state.line_sensors[2] = state.line_sensors[2];
		lcm_state.range = state.range;
		lcm_state.motor_current_left = state.motor_current_left;
		lcm_state.motor_current_right = state.motor_current_right;

		maebot_state_t_publish(lcm, "MAEBOT_STATE", &lcm_state);
	}

	return 0;
}



int main()
{
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
	if(!lcm)
		return 1;

	maebot_command_t_subscribe(lcm, "MAEBOT_COMMAND", &command_handler, NULL);


	int ret;
	printf("Opening port...");

	port = open_port();
	if(port == -1)
	{
		printf("error opening port\n");
		return 1;
	}
	printf("done.\n");
	printf("Configuring port...");
	port = configure_port(port);
	printf("done.\n");

	printf("Listening...\n");

	command_t command;
	const uint32_t msg_sz = HEADER_BYTES + COMMAND_T_BUFFER_BYTES;
	uint8_t buf[msg_sz];

	volatile int8_t left_cmd = 0;
	volatile int8_t right_cmd = 0;

	pthread_t encoder_thread_pid;
	pthread_create(&encoder_thread_pid, NULL, encoder_thread, NULL);

	while(1)
	{
		lcm_handle(lcm);
	}
}

