/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   ForAkmSensor.cpp is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ForAkmSensor.cpp is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <poll.h>
#include <sys/select.h>
#include <dlfcn.h>

#include <pthread.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <setjmp.h>


/************************** Accelerometer and Magnetic Sensor Emulation **************************/

#define ONLY_READINGS

#ifdef ONLY_READINGS

static FILE *accel_readings_fp;
static FILE *magnet_readings_fp;
#define INIT_LOG_READING do {\
				if (!accel_readings_fp) {\
					accel_readings_fp = fopen("/data/accel_readings", "w");\
				}\
				if (!magnet_readings_fp) {\
					magnet_readings_fp = fopen("/data/magnet_readings", "w");\
				}\
			} while(0)
#define LOG_ACCEL_READING (void)(accel_readings_fp && fprintf(accel_readings_fp, "%s\n", send_buf) && fflush(accel_readings_fp))
#define LOG_MAGNET_READING (void)(magnet_readings_fp && fprintf(magnet_readings_fp, "%s\n", send_buf) && fflush(magnet_readings_fp))
#define LOG_SERVER_ACCEL_READING do {\
				if (accel_readings_fp && send_buf[0]) {\
					struct timespec t = { 0 };\
					clock_gettime(CLOCK_REALTIME, &t);\
					unsigned long long int ts = t.tv_sec * 1E9 + t.tv_nsec;\
					fprintf(accel_readings_fp, "[%s] %lluns : ", sensors_name[n], ts);\
					LOG_ACCEL_READING;\
				}\
		} while(0)
#define LOG_SERVER_MAGNET_READING do {\
				if (magnet_readings_fp && send_buf[0]) {\
					struct timespec t = { 0 };\
					clock_gettime(CLOCK_REALTIME, &t);\
					unsigned long long int ts = t.tv_sec * 1E9 + t.tv_nsec;\
					fprintf(magnet_readings_fp, "[%s] %lluns : ", sensors_name[n], ts);\
					LOG_MAGNET_READING;\
				}\
		} while(0)
#else

#define INIT_LOG_READING
#define LOG_ACCEL_READING
#define LOG_MAGNET_READING
#define LOG_SERVER_ACCEL_READING
#define LOG_SERVER_MAGNET_READING

#endif

static FILE *fp;

#define ONLY_ERROR

#ifdef ONLY_ERROR

#define INITIALIZE_LOG do {\
			if (!fp) {\
				fp = fopen("/data/Akm_sensor_log", "w");\
				if (fp) {\
					fprintf(fp, "\n\n***********************************************\n\n");\
					fflush(fp);\
				}\
			}\
		} while(0)
#define ERR(...) (void)(fp && fprintf(fp, "%s %d: ERROR - ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))

#else

#define INITIALIZE_LOG
#define ERR(...)

#endif

// #define ONLY_LOG

#ifdef ONLY_LOG

#define LOG(...) (void)(fp && fprintf(fp, "%s %d: ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))
#define LOG_SERVER(...) (void)(fp && fprintf(fp, "[%s] ", sensors_name[n]), LOG(__VA_ARGS__))
#define LOG_LINE LOG(" ")

#else

#define LOG(...)
#define LOG_SERVER(...)
#define LOG_LINE

#endif

#define ACCEL_READINGS_BUF_SIZE 50 /* 3 readings. */
#define MAGNET_READINGS_BUF_SIZE 100 /* 3 readings. */

#define NUM_SENSORS 2
#define BASE_PORT 5000

enum Sensors { EAccel = 0, EMagnet = 1, };

static const char *sensors_name[NUM_SENSORS] = {
						"Accelerometer",
						"Magnetic",
					      };

static int pipefds[NUM_SENSORS][2] = {	{ -1, -1 },
					{ -1, -1 },
					};

struct accel_poll_data {
	char c; /* 'x', 'y', or 'z' - Indicator. */
	float r; /* One of x, y, or z. */
};

struct magnetic_poll_data {
	char c; /* 'x', 'y', or 'z' - Indicator. */
	float r; /* One of x, y, or z. */
};

union poll_data {
	struct accel_poll_data accel;
	struct magnetic_poll_data magnet;
};

static int listenfd[NUM_SENSORS];
static int connfd[NUM_SENSORS];

static void cleanup(void)
{
	LOG("Cleaning up . . .\n");

	int i = 0;
	while (i < NUM_SENSORS) {
		if (listenfd[i] != -1) {
			close(listenfd[i]);
			listenfd[i] = -1;
		}
		if (connfd[i] != -1) {
			close(connfd[i]);
			connfd[i] = -1;
		}
		i++;
	}
#ifdef ONLY_LOG
	if (fp) {
		fflush(fp);
	}
#endif

	LOG("Cleaned!\n");
}

static void sigsegv_handler(int sig)
{
	LOG("** ATTENTION : SIGSEGV **\n");
	if (fp) {
		fclose(fp);
		fp = NULL;
	}
	(void)rename("/data/Akm_sensor_log", "/data/Akm_sensor_log_ren");
	(void)rename("/data/Akm_readings", "/data/Akm_readings_ren");
	(void)rename("/data/light_sensor_log", "/data/light_sensor_log_ren");
	(void)rename("/data/light_readings", "/data/light_readings_ren");
	(void)rename("/data/proximity_sensor_log", "/data/proximity_sensor_log_ren");
	(void)rename("/data/proximity_readings", "/data/proximity_readings_ren");

	exit(0);
}

static jmp_buf state;
static void sigpipe_handler(int sig)
{
	LOG("** ATTENTION : SIGPIPE **\n");
	longjmp(state, 1);
}

struct device_readings_server_data {
	int port;
	int num;
};

static bool connected[NUM_SENSORS];
static void *sensors_readings_server(void *arg)
{
	(void)signal(SIGPIPE, sigpipe_handler);
	(void)signal(SIGSEGV, sigsegv_handler);

	if (!setjmp(state)) {
		LOG("Saving state . . .\n");
	} else {
		LOG("Restoring state . . .!\n");
		(void)signal(SIGPIPE, sigpipe_handler);
	}

	struct device_readings_server_data *d = reinterpret_cast<struct device_readings_server_data *>(arg);
	int port = d->port;
	int n = d->num;
	free(arg);
	d = NULL;
	arg = NULL;

	LOG_SERVER("** Device server - Started! **\n");

	int *pipefd = pipefds[n];

	LOG_SERVER("Opening socket . . .\n");
	int listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd == -1) {
		ERR("socket - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG_SERVER("Socket opened!\n");

	LOG_SERVER("Setting socket options . . .\n");
	int yes = 1;
	bool socket_opt_set = setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR("setsockopt - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG_SERVER("Set!\n");

	struct sockaddr_in serv_addr = { 0, 0, { 0 }, { 0 } };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_SERVER("Binding socket . . .\n");
	bool bound = bind(listenfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR("bind error - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG_SERVER("Bound!\n");

	LOG_SERVER("Trying to listen . . .\n");
	bool listening = listen(listenfd, 10) != -1;
	if (!listening) {
		ERR("listen error - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG_SERVER("Listening at port : %d!\n", port);

	struct timespec t = { .tv_sec = 0, .tv_nsec = 1ULL, };

	int connfd = -1;

	while (1) {
		connected[n] = false;

		int last_connfd = connfd;
		if (last_connfd != -1) {
			LOG_SERVER("Closing last connection . . .\n");
			close(last_connfd);
			LOG_SERVER("Closed!\n");
			last_connfd = -1;
		}

		LOG_SERVER("Waiting to accept . . .\n");
		connfd = accept(listenfd, (struct sockaddr *)NULL, NULL);
		if (connfd == -1) {
			ERR("accept - %s\n", strerror(errno));
			cleanup();
			return 0;
		}
		LOG_SERVER("Accepted!\n");

		connected[n] = true;

		size_t readings_size = n == EAccel ? ACCEL_READINGS_BUF_SIZE + 1 : MAGNET_READINGS_BUF_SIZE + 1;
		char last_reading[readings_size];
		memset(last_reading, 0, sizeof(last_reading));

		float accel_readings[3] = { 0.0f };
		float magnet_readings[3] = { 0.0f };

		struct pollfd fds = { .fd = pipefd[0], .events = POLLIN, 0, };

		while (1) {
			LOG_SERVER("Polling . . .\n");
			bool polled = poll(&fds, 1, -1) != -1;
			if (!polled) {
				ERR("poll - %s\n", strerror(errno));
				continue;
			}
			LOG_SERVER("Polled!\n");

			bool expected = fds.revents == POLLIN;
			if (!expected) {
				ERR("Unexpected polling event!\n");
				continue;
			}
			LOG_SERVER("Expected poll event!\n");

			union poll_data p;
			memset(&p, 0, sizeof(p));

			char send_buf[readings_size];
			memset(send_buf, 0, sizeof(send_buf));

			LOG_SERVER("Reading poll data . . .\n");
			int bytes_read = read(pipefd[0], &p, sizeof(p));
			if (bytes_read == -1) {
				ERR("read - %s\n", strerror(errno));
				continue;
			} else if (!bytes_read) {
				ERR("Zero bytes read off the pipe.\n");
			} else {
				LOG_SERVER("Successfully read %d bytes off the pipe!\n", bytes_read);

				switch (n) {
					case EAccel:
					{
						accel_readings[p.accel.c - 'x'] = p.accel.r;
						LOG_SERVER("** %c value : %.9f **\n", p.accel.c, p.accel.r);

						snprintf(send_buf, sizeof(send_buf) - 1, "%.9f|%.9f|%.9f",
										accel_readings[0],
										accel_readings[1],
										accel_readings[2]);

						break;
					}
					case EMagnet:
					{
						magnet_readings[p.magnet.c - 'x'] = p.magnet.r;
						LOG("** %c value : %f **\n", p.magnet.c, p.magnet.r);


						snprintf(send_buf, sizeof(send_buf) - 1, "%f|%f|%f",
											magnet_readings[0],
											magnet_readings[1],
											magnet_readings[2]);
						
						break;
					}	
					default:
					{
						LOG_SERVER("NOTE: Unknown poll data with sensor number(%d)\n", n);
						break;
					}
				}
			}

			LOG_SERVER("send_buf: %s\n", send_buf);

			if (strcmp(last_reading, send_buf)) {
				LOG_SERVER("Unique readings!\n");
				int bytes_wrote = write(connfd, send_buf, readings_size);
				if (n == EAccel) {
					LOG_SERVER_ACCEL_READING;
				} else if (n == EMagnet) {
					LOG_SERVER_MAGNET_READING;
				}

				if (bytes_wrote == -1) {
					ERR("write - %s\n", strerror(errno));
					break;
				}
				LOG_SERVER("Wrote %zd bytes!\n", bytes_wrote);

				strcpy(last_reading, send_buf);
			} else {
				LOG_SERVER("Same device reading. Not writing!\n");
			}

			nanosleep(&t, NULL);
		}
			
		nanosleep(&t, NULL);
	}

	return NULL;
}


static pthread_t readings_server_th_id[NUM_SENSORS];
static bool initialize_readings_server(int server_num)
{
	bool initzd = true;

	int i = server_num;

	
	struct device_readings_server_data *d = reinterpret_cast<struct device_readings_server_data *>(malloc(sizeof(*d)));
	if (!d) {
		ERR("malloc - %s\n", strerror(errno));
		goto done;
	}
	d->port = BASE_PORT + i;
	d->num = i;

	{
		pthread_t id = -1;
		bool created = !(errno = pthread_create(&id, NULL, sensors_readings_server, d));
		if (!created) {
			ERR("%s readings server thread *failed* to create - %s\n", sensors_name[i], strerror(errno));
			initzd = false;
		} else {
			LOG("%s readings server thread created!\\m/\n", sensors_name[i]);
			readings_server_th_id[i] = id;
		}
	}

done:	
	return initzd;
}

static bool terminate_accelerometer_readings_server(void)
{
	LOG_LINE;

	bool terminated = true;

	/* Assume that this server never gets terminated. You need the sensor as long as the device is on!
		And hence no destructor called and hence no killing of this server, no resources used by
		it ever gets freed! */

	return terminated;
}

/*****************************************************************************/

// To be part of AkmSensor().
	static bool first_sensor = true;

	INITIALIZE_LOG;
	INIT_LOG_READING;

	if (first_sensor) {
		LOG("First sensor. Initializing!\n");

		(void)signal(SIGSEGV, sigsegv_handler);

		LOG("Piping . . .\n");
		int i = 0;
		while (i < NUM_SENSORS) {
			bool piped = pipe(pipefds[i]) != -1;
			if (piped) {
				LOG("%s Piped!\n", sensors_name[i]);

				LOG("Initializing %s device server . . .\n", sensors_name[i]);
				bool initialized = initialize_readings_server(i);
				if (initialized) {
					LOG("Initialized device server!\n");
				} else {
					ERR("Failed to initialize device server.\n");
				}
			} else {
				ERR("pipe - %s\n", strerror(errno));
			}
			i++;
		}

		first_sensor = false;
	} else {
		LOG("Not the first sensor. Not initializing again!\n");
	}
// To be part of AkmSensor().

// To be part of ~AkmSensor().
	if (terminate_accelerometer_readings_server()) {
		LOG("Accel server terminated successfully!\n");
	} else {
		LOG("Accel server didn't terminate!\n");
	}
// To be part of ~AkmSensor().

// To be part of AkmSensor::processEvent()
// To be part of case event type accel x
	    {
		float r = // Save the acceleration.x reading here.
		if (connected[EAccel]) {
			p.accel.r = r;
			p.accel.c = 'x';
			(void)write(pipefds[EAccel][1], &p, sizeof(p)); // Ignore any error for speed!
			LOG("x: %f\n\n", p.accel.r);
		}
            } 
// To be part of case event type accel y
	    {
		float r = // Save the acceleration.y reading here.
		if (connected[EAccel]) {
			p.accel.r = r;
			p.accel.c = 'y';
			(void)write(pipefds[EAccel][1], &p, sizeof(p)); // Ignore any error for speed!
			LOG("y: %f\n\n", p.accel.r);
		}
            }
// To be part of case event type accel z
	    {
		float r = // Save the acceleration.z reading here.
		if (connected[EAccel]) {
			p.accel.r = r;
			p.accel.c = 'z';
			(void)write(pipefds[EAccel][1], &p, sizeof(p)); // Ignore any error for speed!
			LOG("z: %f\n\n", p.accel.r);
		}
            }
// To be part of case event type magv x
	    {
	        float r = // Save the magnetic.x reading here.
		if (connected[EMagnet]) {
			p.magnet.r = r;
			p.magnet.c = 'x';
			(void)write(pipefds[EMagnet][1], &p, sizeof(p));
			LOG("x: %f\n", r);
		} else {
			LOG("Magnet server not connected!\n");
		}
	    }
// To be part of case event type magv 
	    {
            	float r = // Save the magnetic.y reading here.
		if (connected[EMagnet]) {
			p.magnet.r = r;
			p.magnet.c = 'y';
			(void)write(pipefds[EMagnet][1], &p, sizeof(p));
			LOG("y: %f\n", r);
		} else {
			LOG("Magnet server not connected!\n");
		}
	    }
// To be part of case event type magv z
	    {
            	float r = // Save the magnetic.z reading here.
		if (connected[EMagnet]) {
			p.magnet.r = r;
			p.magnet.c = 'z';
			(void)write(pipefds[EMagnet][1], &p, sizeof(p));
			LOG("z: %f\n", r);
		} else {
			LOG("Magnet server not connected!\n");
		}
	    }
// To be part of AkmSensor::processEvent()

