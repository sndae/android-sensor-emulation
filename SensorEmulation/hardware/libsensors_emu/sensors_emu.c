/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   sensors_emu.c is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   sensors_emu.c is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * sensors_emu.c
 * 
 * Author: Raghavan Santhanam. raghavanil4m@gmail.com, rs3294@columbia.edu
 *
 * Working:
 *
 * This program emulates the 5 real Android sensors - Accelerometer,
 * Magnetic, Light, Proximity, and Gyroscope sensors.
 *
 * The idea behind the emulation is to provide the standard
 * sensor Hardware Abstracion Layer(HAL) interfaces of Android system
 * and fetch the sensors readings for each of the sensors over the
 * network using socket communication APIs in the classic client-server
 * fashion.
 *
 * The provider of the sensor readings over the network can be anything
 * as long as it provides the readings in a network buffer used for a
 * typical socket-communication and has the specific pattern pre-determined
 * for each sensor.
 *
 * Upon receiving these pattern based readings, the respective sensor
 * server will interpret the individual components of the readings as needed
 * for the respective sensor and writes the interperted readings onto
 * a global shared data structure which will be read during a periodically
 * initiated poll event from the Android sensor subsystem.
 *
 * The servers are implemented as separate threads using pthread library.
 */
 

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hardware/sensors.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#include <poll.h>

#include <pthread.h>

#define POLL_DELAY_CONF_FILE "/data/poll_delay.conf"

#define GYRO_NUM_READINGS_AT_ONCE 40
#define ACCEL_NUM_READINGS_AT_ONCE 40
#define NUM_SENSORS 5

#define READINGS_BUF_SIZE 100
#define GYRO_READINGS_BUF_SIZE 50 /* 3 Readings */
#define ACCEL_READINGS_BUF_SIZE 50 /* 3 Readings */

#define ONLY_READING

#ifdef ONLY_READING

static FILE *readings_fp[NUM_SENSORS];
static const char *sensor_readings_files[NUM_SENSORS] = {
							"/data/accel_readings",
							"/data/magnet_readings",
							"/data/light_readings",
							"/data/proximity_readings",
							"/data/gyroscope_readings",
							};
#define INIT_LOG_READING do {\
				int i = 0;\
				while (i < NUM_SENSORS) {\
					if (!readings_fp[i]) {\
						readings_fp[i] = fopen(sensor_readings_files[i], "w");\
					}\
					i++;\
				}\
			} while(0)
#define LOG_READING do {\
			if (readings_fp[n] && readings[0]) {\
				struct timespec t = { 0 };\
				clock_gettime(CLOCK_REALTIME, &t);\
				unsigned long long int ts = t.tv_sec * 1E9 + t.tv_nsec;\
				if (n == EGyro) {\
					int i = 0;\
					while (i < GYRO_NUM_READINGS_AT_ONCE) {\
						fprintf(readings_fp[n], "[%s] %lluns : %s\n", sensors_name[n], ts,\
											readings + i * (GYRO_READINGS_BUF_SIZE + 1));\
						i++;\
					}\
				} else if (n == EAccel) {\
					int i = 0;\
					while (i < ACCEL_NUM_READINGS_AT_ONCE) {\
						fprintf(readings_fp[n], "[%s] %lluns : %s\n", sensors_name[n], ts,\
											readings + i * (ACCEL_READINGS_BUF_SIZE + 1));\
						i++;\
					}\
				} else {\
					fprintf(readings_fp[n], "[%s] %lluns : %s\n", sensors_name[n], ts, readings);\
				}\
				fflush(readings_fp[n]);\
			}\
		} while(0)
#else

#define INIT_LOG_READING
#define LOG_READING

#endif

#define ONLY_ERROR

static FILE *fp;

#ifdef ONLY_ERROR

#define INITIALIZE_ERR_LOG do {\
				if (!fp) {\
					fp = fopen("/data/sensor_log", "a");\
				}\
			} while(0)
#define ERR(...) (void)(fp && fprintf(fp, "%s %d: ERROR - ", __func__, __LINE__) \
								&& fprintf(fp, __VA_ARGS__) && fflush(fp))
#define ERR_SERVER(...) (void)(fp && fprintf(fp, "[%s] %s %d: ERROR - ", sensors_name[n], __func__, __LINE__) \
								&& fprintf(fp, __VA_ARGS__) && fflush(fp))
#define ERR_POLL_PIPE ERR_SERVER
#else

#define INITIALIZE_ERR_LOG
#define ERR(...)
#define ERR_SERVER(...)
#define ERR_POLL_PIPE

#endif

// #define ONLY_LOG

#ifdef ONLY_LOG

#define INITIALIZE_LOG do {\
				if (!fp) {\
					fp = fopen("/data/sensor_log", "a");\
				}\
			} while(0)
#define LOG(...) (void)(fp && fprintf(fp, "%s %d: ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))
#define LOG_SERVER(...) (void)(fp && fprintf(fp, "[%s] - ", sensors_name[n]), LOG(__VA_ARGS__))
#define LOG_POLL_PIPE LOG_SERVER
#define LOG_LINE LOG(" ")

#else

#define INITIALIZE_LOG
#define LOG(...)
#define LOG_SERVER(...)
#define LOG_POLL_PIPE
#define LOG_LINE

#endif /* DEBUG */


#define ID_ACCELERATION (SENSORS_HANDLE_BASE + 0)
#define ID_MAGNETIC (SENSORS_HANDLE_BASE + 1)
#define ID_LIGHT (SENSORS_HANDLE_BASE + 2)
#define ID_PROXIMITY (SENSORS_HANDLE_BASE + 3)
#define ID_GYROSCOPE (SENSORS_HANDLE_BASE + 4)

#define SENSOR_ID(num) (SENSORS_HANDLE_BASE + num)
#define SENSOR_PORT(num) (5000 + num)

#define MAX_SAME_READING_TOLERANCE 4

enum sensors { EAccel = 0, EMagnetic = 1, ELight = 2, EProx = 3, EGyro = 4, };

const char *sensors_name[NUM_SENSORS] =  {
						"Accelerometer",
						"Magnetic",
						"Light",
						"Proximity",
						"Gyroscope",
					};

struct sensor_emulation {
	struct sensors_poll_device_t s_info;
};

static struct sensor_emulation s_emu;

static int dummy_activate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
	LOG_LINE;
	return 0;
}

static int dummy_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	LOG_LINE;
	return 0;
}

static void cleanup(void);

static void sigsegv_handler(int sig)
{
	LOG("** ATTENTION ** SIGSEGV(%d) raised!\n", sig);
	cleanup();
	exit(0);
}

static void sigabrt_handler(int sig)
{
	LOG("** ATTENTION ** SIGABRT(%d) raised!\n", sig);
	cleanup();
	exit(0);
}

static bool initialized;

static int listenfd[NUM_SENSORS];
static int connfd[NUM_SENSORS];
static bool connected[NUM_SENSORS];
sensors_event_t sensor_data[NUM_SENSORS];

static pthread_t emu_readings_server_th_ids[NUM_SENSORS];

static void cleanup_emu_server(int i)
{
	LOG("Cleaning up . . .\n");

	LOG("Closing listeners . . .\n");
	if (listenfd[i] != -1) {
		bool closed = close(listenfd[i]) != -1;
		if (!closed) {
			ERR("close - %s\n", strerror(errno));
		} else {
			LOG("%d - Closed!\n", i);
		}
		listenfd[i] = -1;
	}
	LOG("Closing connections . . .\n");
	if (connfd[i] != -1) {
		bool closed = close(connfd[i]) != -1;
		if (!closed) {
			ERR("close - %s\n", strerror(errno));
		} else {
			LOG("%d - Closed!\n", i);
		}
		connfd[i] = -1;
	}
	connected[i] = false;
}

static void cleanup(void)
{
	LOG("Cleaning . . .\n");
	int i = 0;
	while (i < NUM_SENSORS) {
		cleanup_emu_server(i);
		i++;
	}
	LOG("Cleaned!\n");
}

struct emu_server_data {
	int sensor_id;
	int port;
	int num;
};

// Common server code for 3 of the real sensors : Magnet, Light, and Proximity.
// The readings are received one at a time due to low frequencies of these
// sensors on a real Android device. For remote server scenario, this doesn't
// make any such difference, anyway.
static void *emu_readings_server(void *arg)
{
	struct emu_server_data *data = arg;
	int id = data->sensor_id;
	int port = data->port;
	int n = data->num;

	LOG_SERVER("\n\n** Emulator server for %s - Started! **\n", sensors_name[n]);
	LOG_SERVER("[%d] Data : %p\n", n, (void *)data);
	LOG_SERVER("[%d] Sensor id : %d\n", n, id);
	LOG_SERVER("[%d] Port : %d\n\n", n, port);

	free(data);
	data = NULL;

	LOG_SERVER("Opening socket . . .\n");
	listenfd[n] = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd[n] == -1) {
		ERR_SERVER("socket - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Opened!\n");

	LOG_SERVER("Setting options . . .\n");
	int yes = 1;
	bool socket_opt_set = setsockopt(listenfd[n], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR_SERVER("setsockopt - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Set!\n");

	struct sockaddr_in serv_addr = { 0 };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_SERVER("Binding socket . . .\n");
	bool bound = bind(listenfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR_SERVER("bind - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Bound!\n");

	LOG_SERVER("Trying to listen . . .\n");
	bool listening = listen(listenfd[n], 10) != -1;
	if (!listening) {
		ERR_SERVER("listen - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Listening!\n");

	unsigned long long int delay_ns = 100ULL;
	struct timespec t = { .tv_sec = 0, .tv_nsec = delay_ns, };

	connfd[n] = -1;

	while (1) {
		connected[n] = false;

		LOG_SERVER("Waiting to accept . . .\n");
		connfd[n] = accept(listenfd[n], (struct sockaddr *)NULL, NULL);
		if (connfd[n] == -1) {
			ERR_SERVER("accept - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Accepted!\n");

		int same_r_num = 0;	
		char last_readings[READINGS_BUF_SIZE + 1] = "";
		while (1) {
			char readings[READINGS_BUF_SIZE + 1] = "";

			LOG_SERVER("Receiving . . .\n");
			ssize_t bytes_received = recvfrom(connfd[n], readings, READINGS_BUF_SIZE + 1, MSG_WAITALL, NULL, 0);
			LOG_READING;

			if (bytes_received == -1) {
				ERR_SERVER("recvfrom - %s\n", strerror(errno));
				break;
			}
			LOG_SERVER("Received %lu bytes!\n", bytes_received);
			LOG_SERVER("Readings: %s\n", readings);

			if (bytes_received == 0) {
				LOG_SERVER("Zero bytes received! Likely a faulty socket. Accepting again.\n");
				break;
			}

			bool device_locked = !readings[0];
			if (device_locked) {
				LOG("Device is likely in locked state!\n");
				continue;
			}

			connected[n] = true;

			bool same_r = !strcmp(readings, last_readings);
			if (same_r) {
				same_r_num++;
				if (same_r_num == MAX_SAME_READING_TOLERANCE) {
					LOG("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					ERR("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					break;
				}
			}
			strcpy(last_readings, readings);

			sensor_data[n].sensor = id;

			switch(id) {
				case ID_MAGNETIC:
				{
					LOG_SERVER("Sensor: Magnetic\n");
					char *f_d = strchr(readings, '|');
					*f_d = '\0';
					char *s_d = strchr(f_d + 1, '|');
					*s_d = '\0';

					sscanf(readings, "%f", &sensor_data[n].magnetic.x);
					sscanf(f_d + 1, "%f", &sensor_data[n].magnetic.y);
					sscanf(s_d + 1, "%f", &sensor_data[n].magnetic.z);
					break;
				}
				case ID_LIGHT:
				{
					LOG_SERVER("Sensor: Light\n");
					sscanf(readings, "%f", &sensor_data[n].light);
					break;
				}
				case ID_PROXIMITY:
				{
					LOG_SERVER("Sensor: Proximity\n");
					sscanf(readings, "%f", &sensor_data[n].distance);
					break;
				}
				default:
				{
					LOG_SERVER("** Unknown sensor id(%d)\n", id);
					break;
				}
			}


			nanosleep(&t, NULL);
		}

		close(connfd[n]);
		connfd[n] = -1;
			
		nanosleep(&t, NULL);
	}
	
done:
	cleanup_emu_server(n);	
	LOG_SERVER("** Emulator server for %s - Terminated! **\n", sensors_name[n]);

	return NULL;
}


// In order to stay up to the speedy gyroscope sensor data from the real Android device
// when used, gyroscope server has this separate unique code. The special thing in this
// code is that instead of fetching one reading at a time over the network, a bunch of
// readings are fetched over the network. The source of the readings can be a real
// Android device or a remote server. For remote server scenario, this speed up may
// not make much difference.
static int gyro_pipefd[2] = { -1, -1 };
static void *emu_gyro_readings_server(void *arg)
{
	struct emu_server_data *data = arg;
	int id = data->sensor_id;
	int port = data->port;
	int n = data->num;

	LOG_SERVER("\n\n** Emulator server for %s - Started! **\n", sensors_name[n]);
	LOG_SERVER("[%d] Data : %p\n", n, (void *)data);
	LOG_SERVER("[%d] Sensor id : %d\n", n, id);
	LOG_SERVER("[%d] Port : %d\n\n", n, port);

	free(data);
	data = NULL;

	LOG_SERVER("Opening socket . . .\n");
	listenfd[n] = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd[n] == -1) {
		ERR_SERVER("socket - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Opened!\n");

	LOG_SERVER("Setting options . . .\n");
	int yes = 1;
	bool socket_opt_set = setsockopt(listenfd[n], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR_SERVER("setsockopt - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Set!\n");

	struct sockaddr_in serv_addr = { 0 };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_SERVER("Binding socket . . .\n");
	bool bound = bind(listenfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR_SERVER("bind - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Bound!\n");

	LOG_SERVER("Trying to listen . . .\n");
	bool listening = listen(listenfd[n], 10) != -1;
	if (!listening) {
		ERR_SERVER("listen - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Listening!\n");

	struct timespec t = { .tv_sec = 0, .tv_nsec = 10ULL, };

	connfd[n] = -1;

	while (1) {
		connected[n] = false;

		LOG_SERVER("Waiting to accept . . .\n");
		connfd[n] = accept(listenfd[n], (struct sockaddr *)NULL, NULL);
		if (connfd[n] == -1) {
			ERR_SERVER("accept - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Accepted!\n");

		connected[n] = true;

		char last_readings[GYRO_NUM_READINGS_AT_ONCE * (GYRO_READINGS_BUF_SIZE + 1)] = "";
		int same_r_num = 0;
		while (1) {
			char readings[GYRO_NUM_READINGS_AT_ONCE * (GYRO_READINGS_BUF_SIZE + 1)] = "";

			LOG_SERVER("Receiving . . .\n");
			ssize_t bytes_received = recvfrom(connfd[n], readings,
								GYRO_NUM_READINGS_AT_ONCE * (GYRO_READINGS_BUF_SIZE + 1), MSG_WAITALL, NULL, 0);
			LOG_READING;

			if (bytes_received == -1) {
				ERR_SERVER("recvfrom - %s\n", strerror(errno));
				break;
			}
			LOG_SERVER("Received %lu bytes!\n", bytes_received);
			LOG_SERVER("Readings: %s\n", readings);

			if (bytes_received == 0) {
				LOG_SERVER("Zero bytes received! Likely a faulty socket. Accepting again.\n");
				break;
			}

			bool device_locked = !readings[0];
			if (device_locked) {
				LOG("Device is likely in locked state!\n");
				continue;
			}

			bool same_r = !strcmp(readings, last_readings);
			if (same_r) {
				same_r_num++;
				if (same_r_num == MAX_SAME_READING_TOLERANCE) {
					LOG_SERVER("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					ERR_SERVER("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					break;
				}
			}
			strcpy(last_readings, readings);

			LOG_SERVER("Writing onto gyro pipe . . .\n");
			int bytes_wrote = write(gyro_pipefd[1], readings, sizeof(readings));
			if (bytes_wrote == -1) {
				ERR_SERVER("write - failed to write onot gyroscope pipe - %s\n", strerror(errno));
			} else {
				LOG_SERVER("Wrote %d bytes onto gyroscope pipe!\n", bytes_wrote);
			}	

			nanosleep(&t, NULL);
		}

		close(connfd[n]);
		connfd[n] = -1;
			
		nanosleep(&t, NULL);
	}
	
done:
	cleanup_emu_server(n);	
	LOG_SERVER("** Emulator server for %s - Terminated! **\n", sensors_name[n]);

	return NULL;
}

// Since accelerometer is observed to be next speedy sensor other than gyroscope,
// simple read-one-at-a-time doesn't work for accelerometer readings. Hence, to
// mitigate the impact on accelerometer server by the gyroscope server, the
// accelerometer server code is also using gyroscope server logic of getting a
// bunch of readings in one go. The notable difference between accelerometer
// and gyroscope servers is that the number of readings being got in one shot.
// But, the real reason to have seemingly duplicated code for these two is from
// the futuristic thinking that one can customize these servers without having
// to have a check each time whether it's gyroscope or accelerometer in every
// iteration of the recvfrom() in the innermost loop of these servers.
static int accel_pipefd[2] = { -1, -1 };
static void *emu_accel_readings_server(void *arg)
{
	struct emu_server_data *data = arg;
	int id = data->sensor_id;
	int port = data->port;
	int n = data->num;

	LOG_SERVER("\n\n** Emulator server for %s - Started! **\n", sensors_name[n]);
	LOG_SERVER("[%d] Data : %p\n", n, (void *)data);
	LOG_SERVER("[%d] Sensor id : %d\n", n, id);
	LOG_SERVER("[%d] Port : %d\n\n", n, port);

	free(data);
	data = NULL;

	LOG_SERVER("Opening socket . . .\n");
	listenfd[n] = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd[n] == -1) {
		ERR_SERVER("socket - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Opened!\n");

	LOG_SERVER("Setting options . . .\n");
	int yes = 1;
	bool socket_opt_set = setsockopt(listenfd[n], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR_SERVER("setsockopt - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Set!\n");

	struct sockaddr_in serv_addr = { 0 };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_SERVER("Binding socket . . .\n");
	bool bound = bind(listenfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR_SERVER("bind - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Bound!\n");

	LOG_SERVER("Trying to listen . . .\n");
	bool listening = listen(listenfd[n], 10) != -1;
	if (!listening) {
		ERR_SERVER("listen - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Listening!\n");

	// struct timespec t = { .tv_sec = 0, .tv_nsec = 1000ULL, }; // For remote server
	struct timespec t = { .tv_sec = 0, .tv_nsec = 100ULL, }; // For real device

	connfd[n] = -1;

	while (1) {
		connected[n] = false;

		LOG_SERVER("Waiting to accept . . .\n");
		connfd[n] = accept(listenfd[n], (struct sockaddr *)NULL, NULL);
		if (connfd[n] == -1) {
			ERR_SERVER("accept - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Accepted!\n");

		connected[n] = true;

		char last_readings[ACCEL_NUM_READINGS_AT_ONCE * (ACCEL_READINGS_BUF_SIZE + 1)] = "";
		int same_r_num = 0;
		while (1) {
			char readings[ACCEL_NUM_READINGS_AT_ONCE * (ACCEL_READINGS_BUF_SIZE + 1)] = "";

			LOG_SERVER("Receiving . . .\n");
			ssize_t bytes_received = recvfrom(connfd[n], readings,
								ACCEL_NUM_READINGS_AT_ONCE * (ACCEL_READINGS_BUF_SIZE + 1),
														MSG_WAITALL, NULL, 0);
			LOG_READING;

			if (bytes_received == -1) {
				ERR_SERVER("recvfrom - %s\n", strerror(errno));
				break;
			}
			LOG_SERVER("Received %lu bytes!\n", bytes_received);
			LOG_SERVER("Readings: %s\n", readings);

			if (bytes_received == 0) {
				LOG_SERVER("Zero bytes received! Likely a faulty socket. Accepting again.\n");
				break;
			}

			bool device_locked = !readings[0];
			if (device_locked) {
				LOG("Device is likely in locked state!\n");
				continue;
			}

			bool same_r = !strcmp(readings, last_readings);
			if (same_r) {
				same_r_num++;
				if (same_r_num == MAX_SAME_READING_TOLERANCE) {
					LOG("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					ERR("Same reading for %d times. Fishy! Resetting connection . . .\n", same_r_num);
					break;
				}
			}
			strcpy(last_readings, readings);

			LOG_SERVER("Writing onto gyro pipe . . .\n");
			int bytes_wrote = write(accel_pipefd[1], readings, sizeof(readings));
			if (bytes_wrote == -1) {
				ERR_SERVER("write - failed to write onto accelerometer pipe - %s\n", strerror(errno));
			} else {
				LOG_SERVER("Wrote %d bytes onto accelerometer pipe!\n", bytes_wrote);
			}

			nanosleep(&t, NULL);
		}

		close(connfd[n]);
		connfd[n] = -1;
			
		nanosleep(&t, NULL);
	}
	
done:
	cleanup_emu_server(n);	
	LOG_SERVER("** Emulator server for %s - Terminated! **\n", sensors_name[n]);

	return NULL;
}

static bool unblock_pipes(int pipefd[])
{
	bool unblocked = false;
	
	bool unblocked_front = false;
	bool unblocked_back = false;

	LOG("Gyro piped!\n");

	LOG("Unblocking pipe-front . . .\n");
	int front_flags = fcntl(pipefd[0], F_GETFL, 0);
	bool front_fcntld = front_flags != -1;
	if (front_fcntld) {
		LOG("Got pipe-front flags!\n");

		bool unblocked = fcntl(pipefd[0], F_SETFL, front_flags | O_NONBLOCK) != -1;
		if (unblocked) {
			LOG("Pipe-front unblocked!\n");
		} else {
			ERR("fcntl - failed to unblock pipe-front - %s\n", strerror(errno));
		}

		unblocked_front = true;
		LOG("Unblocked!\n");
	} else {
		ERR("fcntl - failed to get pipe-front flags - %s\n", strerror(errno));
		LOG("Failed to unblock.\n");
	}

	LOG("Unblocking pipe-back . . .\n");
	bool back_flags = fcntl(pipefd[1], F_GETFL, 0);
	bool back_fcntld = back_flags != -1;
	if (back_fcntld) {
		LOG("Got pipe-back flags!\n");

		bool unblocked = fcntl(pipefd[1], F_SETFL, back_flags | O_NONBLOCK) != -1;
		if (unblocked) {
			LOG("Pipe-back unblocked!\n");
		} else {
			ERR("fcntl - failed to unblock pipe-back - %s\n", strerror(errno));
		}

		unblocked_back = true;
		LOG("Unblocked!\n");
	} else {
		ERR("fcntl - failed to get pipe-back flags - %s\n", strerror(errno));
		LOG("Failed to unblock.\n");
	}

	unblocked = unblocked_front && unblocked_back;

	return unblocked;
}

static bool create_emu_server_threads(void)
{
	static bool fine = true;

	int i = 0;

	LOG("Creating emulator servers . . .\n");

	while (i < NUM_SENSORS) {
		LOG("Creating thread . . .\n");
		pthread_t id = -1;
		struct emu_server_data *d = malloc(sizeof(*d));
		if (!d) {
			ERR("malloc - %s\n", strerror(errno));
			goto done;
		}

		memset(d, 0, sizeof(*d));
		d->sensor_id = SENSOR_ID(i);
		d->port =  SENSOR_PORT(i);
		d->num = i;

		int ret = -1;
		if (i == EAccel) {
			bool piped = pipe(accel_pipefd) != -1; // No pipe2() in Android. So, pipe() and unblock by self!
			if (piped) {
				LOG("Accelerometer piped!\n");
				bool unblocked = unblock_pipes(accel_pipefd);
				if (unblocked) {
					ret = pthread_create(&id, NULL, emu_accel_readings_server, d); // No way of stopping
								// this thread once started, as far as I know!
								// So, there is no such function which cleans up this
								// created thread!
				}
			} else {
				ERR("pipe - Accelerometer failed to pipe - %s\n", strerror(errno));
			}
		} else if (i == EGyro) {
			bool piped = pipe(gyro_pipefd) != -1; // No pipe2() in Android. So, pipe() and unblock by self!
			if (piped) {
				LOG("Gyroscope piped!\n");
				bool unblocked = unblock_pipes(gyro_pipefd);
				if (unblocked) {
					ret = pthread_create(&id, NULL, emu_gyro_readings_server, d); // No way of stopping
								// this thread once started, as far as I know!
								// So, there is no such function which cleans up this
								// created thread!
				}
			} else {
				ERR("pipe - Gyroscope failed to pipe - %s\n", strerror(errno));
			}
		} else {
			ret = pthread_create(&id, NULL, emu_readings_server, d); // No way of stopping
						// this thread once started, as far as I know!
						// So, there is no such function which cleans up this
						// created thread!
		}

		bool created = ret == 0;
		if (!created) {
			fine = false;
			errno = ret;
			ERR("pthread_create - Thread *failed* to create - %s\n", strerror(errno));
		} else {
			LOG("Thread created!\\m/\n");
			emu_readings_server_th_ids[i] = id;
		}
		LOG("Created!\n");

		i++;
	}

	LOG("Created!\n");

done:

	return fine;
}

static bool initialize_emu_readings_server(void)
{
	bool success = create_emu_server_threads();

	return success;
}

// Only for triplets.
static bool poll_sensor_pipe(sensors_event_t sensor_data[], int sensor, int pipefd[], int readings_size,
											int sensor_j, int64_t ts)
{
	bool pipe_polled = false;

	int n = sensor;

	struct pollfd pipefds = { .fd = pipefd[0], .events = POLLIN, 0, };

	LOG_POLL_PIPE("Polling for pipe data . . .\n");
	int timeout_ms = 1;
	int ret = poll(&pipefds, 1, timeout_ms);
	bool timed_out = ret == 0;
	bool polled = ret != -1;

	if (!timed_out && polled) {
		LOG_POLL_PIPE("Poll event. Reading poll data . . .\n");

		char readings[readings_size];
		memset(readings, 0, sizeof(readings));

		(void)read(pipefd[0], readings, sizeof(readings));
		if (readings[0]) {
			LOG_POLL_PIPE("pipe reading : %s\n", readings);

			char *f_d = strchr(readings, '|');
			*f_d = '\0';
			char *s_d = strchr(f_d + 1, '|');
			*s_d = '\0';

			sscanf(readings, "%f", &sensor_data[sensor_j].data[0]);
			sscanf(f_d + 1, "%f", &sensor_data[sensor_j].data[1]);
			sscanf(s_d + 1, "%f", &sensor_data[sensor_j].data[2]);
			sensor_data[sensor_j].sensor = sensor;
			sensor_data[sensor_j].timestamp = ts;

			LOG_POLL_PIPE("Read poll event data: %.9f|%.9f|%.9f\n", sensor_data[sensor_j].data[0],
										sensor_data[sensor_j].data[1],
										sensor_data[sensor_j].data[2]);

			pipe_polled = true;
		} else {
			LOG_POLL_PIPE("Read poll event data but EMPTY!\n");
		}
	} else if (timed_out) {
		LOG_POLL_PIPE("Timed out after %d ms!\n", timeout_ms);
	} else if (!polled) {
		LOG_POLL_PIPE("Poll failed!\n");
		ERR_POLL_PIPE("poll - pipe data failed to be polled - %s\n", strerror(errno));
	}

	return pipe_polled;
}

//static int64_t delay_us = 80000; // For remote server scenario. Yes, this much delay is needed
					// as the remote server interacts very fast!
static int64_t delay_us = 1000; // For real paired device -- Working and tested several
				// times for real-time nature. DON'T CHANGE UNLESS YOU'RE
				// SURE WHAT YOU'RE DOING!
static int dummy_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count)
{
	LOG("Sensor event - Polling(just reading from address %p) . . .\n", (void *)data);

	int num_events = 0;

	usleep(delay_us);

	struct timespec t = { 0 };
	bool timed = clock_gettime(CLOCK_MONOTONIC, &t) != -1;
	if (!timed) {
		ERR("clock_gettime - %s\n", strerror(errno));
		goto done;
	}	

	int64_t ts = (int64_t)t.tv_sec * 1000000000ULL + (int64_t)t.tv_nsec + delay_us * 1000;

	int j = 0;
	int i = 0;

	if (connected[EAccel]) {
		LOG("Accelerometer server is connected!\n");
		bool polled = poll_sensor_pipe(data, EAccel, accel_pipefd, ACCEL_READINGS_BUF_SIZE + 1, j, ts);
		if (polled) {
			LOG("Accelerometer pipe successfully polled and read.\n");
			num_events++;
			j++;
		}
	} else {
		LOG("Accelerometer server not connected.\n");
	}

	i = 1;
	while (i < NUM_SENSORS - 1) {
		if (connected[i]) {
			connected[i] = false; // If in case, the device has closed
						// the application querying the
						// specific sensor, then this
						// will prevent any false readings
						// from getting given as the poll
						// data during the next poll.
						//
						// There will be a race condition
						// between this polling thread
						// and the server thread. But, that's
						// fine -- speed matters!

			data[j] = sensor_data[i];
			data[j].timestamp = ts;
		
			j++;
			
			num_events++;
		}
		i++;
	}

	if (connected[EGyro]) {
		LOG("Gyroscope server is connected!\n");
		bool polled = poll_sensor_pipe(data, EGyro, gyro_pipefd, GYRO_READINGS_BUF_SIZE + 1, j, ts);
		if (polled) {
			LOG("Gyroscope pipe successfully polled and read.\n");
			num_events++;
			j++;
		}
	} else {
		LOG("Gyroscope server not connected.\n");
	}

	LOG("Polled!\n");

done:

	LOG("Number of events : %d\n", num_events);

	return num_events; // Number of readings/events.
}

static void sensor_info_init(const struct hw_module_t *module, struct hw_device_t **device)
{
	LOG("Initializing sensor info . . .\n");

	s_emu.s_info.activate = dummy_activate;
	s_emu.s_info.setDelay = dummy_setDelay;
	s_emu.s_info.poll = dummy_poll;

	s_emu.s_info.common.tag = HARDWARE_DEVICE_TAG;
	s_emu.s_info.common.version = 0;
	s_emu.s_info.common.module  = (struct hw_module_t *)module;
	*device = &s_emu.s_info.common;

	LOG("Initialized!\n");
}

static void sensor_simulation_core(const struct hw_module_t *module, struct hw_device_t **device)
{
	LOG("Setting up sensor details . . .\n");

	sensor_info_init(module, device);

	LOG("Set!\n");
}

static int initialize_sensor(const struct hw_module_t *module, const char *id, struct hw_device_t **device)
{
	(void)signal(SIGSEGV, sigsegv_handler);
	(void)signal(SIGABRT, sigabrt_handler);

	INITIALIZE_LOG;
	INITIALIZE_ERR_LOG;
	INIT_LOG_READING;

	LOG("Opening sensor with id : %s\n", id);

	LOG("Poll delay configuration file(%s) there?\n", POLL_DELAY_CONF_FILE);
	FILE *poll_delay_conf_fp = fopen(POLL_DELAY_CONF_FILE, "r");
	if (poll_delay_conf_fp) {
		LOG("Yes!\n");

		int64_t delay_spec = 0;

		errno = 0;
		bool success = fscanf(poll_delay_conf_fp, "%lld", &delay_spec) == 1;

		if (success) {
			if (delay_spec > 0) {
				LOG("Using poll delay specified : %lld micro sec(s)\n", delay_spec);
				delay_us = delay_spec;
			} else {
				LOG("Invalid poll delay specified : %lld\n", delay_spec);
				LOG("Using default poll delay : %lld micro sec(s)\n", delay_us);
			}
		} else {
			ERR("fscanf - Failed to read poll delay - %s\n", strerror(errno));
			LOG("Invalid value read : %lld\n", delay_spec);
			LOG("Using default poll delay : %lld micro sec(s)\n", delay_us);
		}

		fclose(poll_delay_conf_fp);
		poll_delay_conf_fp = NULL;
	} else {
		LOG("No!\n");
		ERR("fopen - Failed to open - %s\n", strerror(errno));
	}

	int i = 0;
	while (i < NUM_SENSORS) {
		connfd[i] = listenfd[i] = -1;
		emu_readings_server_th_ids[i] = -1;
		i++;
	}

	if (!initialized) {
		LOG("Not initialized before. Trying to initialize . . .\n");
		cleanup();
		initialized = initialize_emu_readings_server();
		if (!initialized) {
			LOG("Emu server failed to initialize.\n");
		} else {
			LOG("Emu server initialized!\n");
		}
	} else {
		LOG("Emu server already initialized!\n");
	}

	sensor_simulation_core(module, device);

	return 0;
}

/* Below constants are taken from the values specified in the code for real Android device. */

// Accelerometer
#define RANGE_A (2 * GRAVITY_EARTH)
#define LSB (64.0f)
#define NUMOFACCDATA (8.0f)
#define CONVERT_A (GRAVITY_EARTH / LSB / NUMOFACCDATA)

// Magnetic
#define CONVERT_M (1.0f/16.0f)

// Gyroscope
#define RANGE_GYRO (2000.0f*(float)M_PI/180.0f)
#define CONVERT_GYRO ((70.0f / 1000.0f) * ((float)M_PI / 180.0f))

// The numerical values used below for power etc are from that of the real Android device code.
static struct sensor_t sensor_list[] = {
	{
		.name = "Accelerometer Sensor Emulation!!!!",
		.vendor = "Columbia University - NYC - Thesis - Raghavan Santhanam",
		.version = 1,
		.handle = ID_ACCELERATION,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.maxRange = RANGE_A,
		.resolution = CONVERT_A,
		.power = 3.0f,
		.minDelay = 20000,
		.reserved = { },
	},
	{
		.name = "Magnetic sensor Emulation!!!!",
		.vendor = "Columbia University - NYC - Thesis - Raghavan Santhanam",
		.version = 1,
		.handle = ID_MAGNETIC,
		.type = SENSOR_TYPE_MAGNETIC_FIELD,
		.maxRange = 2000.0f,
		.resolution = CONVERT_M,
		.power = 6.8f,
		.minDelay = 16667,
		.reserved = { },
	},
	{
		.name = "Light sensor Emulation!!!!",
		.vendor = "Columbia University - NYC - Thesis - Raghavan Santhanam",
		.version = 1,
		.handle = ID_LIGHT,
		.type = SENSOR_TYPE_LIGHT,
		.maxRange = 0.0f,
		.resolution = 1.0f,
		.power = 0.75f,
		.minDelay = 0,
		.reserved = { },
	},
	{
		.name = "Proximity sensor Emulation!!!!",
		.vendor = "Columbia University - NYC - Thesis - Raghavan Santhanam",
		.version = 1,
		.handle = ID_PROXIMITY,
		.type = SENSOR_TYPE_PROXIMITY,
		.maxRange = 5.0f,
		.resolution = 5.0f,
		.power = 0.75f,
		.minDelay = 0,
		.reserved = { },
	},
	{
		.name = "Gyroscope sensor Emulation!!!!",
		.vendor = "Columbia University - NYC - Thesis - Raghavan Santhanam",
		.version = 1,
		.handle = ID_GYROSCOPE,
		.type = SENSOR_TYPE_GYROSCOPE,
		.maxRange = RANGE_GYRO,
		.resolution = CONVERT_GYRO,
		.power = 6.1f,
		.minDelay = 1190,
		.reserved = { },
	},
};

static int list_of_sensors(struct sensors_module_t *module, struct sensor_t const **list)
{
	LOG("Getting sensor list . . .\n");

	LOG("module: %p\n", (void *)module);

	*list = sensor_list;

	LOG("Got!\n");

	return sizeof(sensor_list) / sizeof(struct sensor_t);
}

static struct hw_module_methods_t setup_sensors = {
	.open = initialize_sensor,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	common: {
		.tag = HARDWARE_MODULE_TAG,
		.version_major = 2,
		.version_minor = 3,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "Sensor Emulation",
		.author = "Raghavan Santhanam",
		.methods = &setup_sensors,
		.dso = 0,
		.reserved = { }
	},
	get_sensors_list: list_of_sensors,
};


