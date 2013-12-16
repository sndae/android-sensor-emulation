/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   ForProximitySensor.cpp is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ForProximitySensor.cpp is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>


/************************** Proximity Sensor Emulation **************************/

#define ONLY_READING

#ifdef ONLY_READING

static FILE *reading_fp;
#define INIT_LOG_READING do {\
				reading_fp = fopen("/data/proximity_readings", "w");\
			} while(0)
#define LOG_READING do {\
			if (reading_fp && send_buf[0]) {\
				struct timespec t = { 0 };\
				clock_gettime(CLOCK_REALTIME, &t);\
				unsigned long long int ts = t.tv_sec * 1E9 + t.tv_nsec;\
				fprintf(reading_fp, "[Proximity] %lluns : %s\n", ts, send_buf);\
				fflush(reading_fp);\
			}\
		} while(0)
#else

#define INIT_LOG_READING
#define LOG_READING

#endif

static FILE *fp;
#define ONLY_ERROR

#ifdef ONLY_ERROR

#define INITIALIZE_LOG do {\
			fp = fopen("/data/proximity_sensor_log", "w");\
		} while(0)
#define ERR(...) (void)(fp && fprintf(fp, "%s %d: ERROR - ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))

#else

#define INITIALIZE_LOG
#define ERR(...)

#endif

// #define ONLY_LOG

#ifdef ONLY_LOG


#define LOG(...) (void)(fp && fprintf(fp, "%s %d: ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))
#define LOG_LINE LOG(" ")

#else

#define LOG(...)
#define LOG_LINE

#endif

#define READINGS_BUF_SIZE 100 /* 1 readings. */
#define PROXIMITY_SERVER_PORT 5003

static int pipefd[2] = { -1, -1 };

struct poll_data {
	float d;
};

static int listenfd = -1;
static int connfd = -1;

static void cleanup(void)
{
	LOG("Cleaning up . . .\n");

	if (listenfd != -1) {
		close(listenfd);
	}
	if (connfd != -1) {
		close(connfd);
	}
#ifdef ONLY_LOG
	if (fp) {
		fflush(fp);
	}
#endif

	LOG("Cleaned!\n");
}

static bool connected;
static void *proximity_readings_server(void *arg)
{
	LOG("** Proximity device server - Started! **\n");

	(void)arg;

	LOG("Opening socket . . .\n");
	int listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd == -1) {
		ERR("socket - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG("Socket opened!\n");

	LOG("Setting socket options . . .\n");
	int yes = 1;
	bool socket_opt_set = setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR("setsockopt - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG("Set!\n");

	struct sockaddr_in serv_addr = { 0, 0, { 0 }, { 0 } };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(PROXIMITY_SERVER_PORT);

	int port = PROXIMITY_SERVER_PORT;

	LOG("Binding socket . . .\n");
	bool bound = bind(listenfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR("bind error - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG("Bound!\n");

	LOG("Trying to listen . . .\n");
	bool listening = listen(listenfd, 10) != -1;
	if (!listening) {
		ERR("listen error - %s\n", strerror(errno));
		cleanup();
		return 0;
	}
	LOG("Listening at port : %d!\n", port);

	struct timespec t = { .tv_sec = 0, .tv_nsec = 1ULL, };

	int connfd = -1;

	while (1) {
		connected = false;

		int last_connfd = connfd;
		if (last_connfd != -1) {
			LOG("Closing last connection . . .\n");
			close(last_connfd);
			LOG("Closed!\n");
			last_connfd = -1;
		}

		LOG("Waiting to accept . . .\n");
		connfd = accept(listenfd, (struct sockaddr *)NULL, NULL);
		if (connfd == -1) {
			ERR("accept - %s\n", strerror(errno));
			cleanup();
			return 0;
		}
		LOG("Accepted!\n");

		connected = true;

		char last_reading[READINGS_BUF_SIZE + 1] = "";

		float reading = 0.0f;

		struct pollfd fds = { .fd = pipefd[0], .events = POLLIN, 0, };

		while (1) {
			LOG("Polling . . .\n");
			bool polled = poll(&fds, 1, -1) != -1;
			if (!polled) {
				ERR("poll - %s\n", strerror(errno));
				continue;
			}
			LOG("Polled!\n");

			bool expected = fds.revents == POLLIN;
			if (!expected) {
				ERR("Unexpected polling event!\n");
				continue;
			}
			LOG("Expected poll event!\n");

			LOG("Reading poll data . . .\n");
			struct poll_data p = { 0.0f };
			int bytes_read = read(pipefd[0], &p, sizeof(p));
			if (bytes_read == -1) {
				ERR("read - %s\n", strerror(errno));
				continue;
			} else if (!bytes_read) {
				ERR("Zero bytes read off the pipe.\n");
			} else {
				reading = p.d;
				LOG("** Distance value : %f **\n", p.d);
				LOG("Successfully read %d bytes off the pipe!\n", bytes_read);
			}

			char send_buf[READINGS_BUF_SIZE + 1] = "";
			snprintf(send_buf, sizeof(send_buf) - 1, "%f", reading); // The version of
								// libc.so in the device I am using,
								// seems like having some buffer
								// overflow problem! It was totally
								// annoying to confirm the same and
								// finally after a day long of thinking
								// about the alternatives for sprintf(),
								// I ended up using snprintf() which
								// worked out and hence it was confirmed
								// to have the guessed buffer overflow
								// problem.	

			LOG("send_buf: %s\n", send_buf);

			if (strcmp(last_reading, send_buf)) {
				LOG("Unique readings!\n");
				int bytes_wrote = write(connfd, send_buf, READINGS_BUF_SIZE + 1);
				LOG_READING;

				if (bytes_wrote == -1) {
					ERR("write - %s\n", strerror(errno));
					break;
				}
				LOG("Wrote %zd bytes!\n", bytes_wrote);

				strcpy(last_reading, send_buf);
			} else {
				LOG("Same device reading. Not writing!\n");
			}

			nanosleep(&t, NULL);
		}
			
		nanosleep(&t, NULL);
	}

	return NULL;
}

static pthread_t proximity_readings_server_th_id = -1;
static bool initialize_proximity_readings_server(void)
{
	pthread_t id = -1;
	bool created = !(errno = pthread_create(&id, NULL, proximity_readings_server, NULL));
	if (!created) {
		ERR("Proximity readings server thread *failed* to create - %s\n", strerror(errno));
	} else {
		LOG("Proximity readings server thread created!\\m/\n");
		proximity_readings_server_th_id = id;
	}
	
	return created;
}

static bool terminate_proximity_readings_server(void)
{
	LOG_LINE;

	bool terminated = true;

	/* Assume that this server never gets terminated. You need the sensor as long as the device is on!
		And hence no destructor called and hence no killing of this server, no resources used by
		it ever gets freed! */

	return terminated;
}



/*****************************************************************************/

/*****************************************************************************/

// To be part of ProximitySensor().
	INITIALIZE_LOG;
	INIT_LOG_READING;

	LOG("Piping . . .\n");
	bool piped = pipe(pipefd) != -1;
	if (piped) {
		LOG("Light Piped!\n");

		LOG("Initializing Proximity device server . . .\n");
		bool initialized = initialize_proximity_readings_server();
		if (initialized) {
			LOG("Initialized device server!\n");
		} else {
			ERR("Failed to initialize device server.\n");
		}
	} else {
		ERR("pipe - %s\n", strerror(errno));
	}
// To be part of ProximitySensor().

// To be part of ~ProximitySensor().
ProximitySensor::~ProximitySensor() {
	if (terminate_proximity_readings_server()) {
		LOG("Proximity server terminated successfully!\n");
	} else {
		LOG("Proximity server didn't terminate!\n");
	}		
// To be part of ~ProximitySensor().

// To be part of setInitialState().
// To be part of the if() block of ioctl check.
        float d = // Save the distance value.

	LOG("Setting initial state . . .\n");

	if (connected) {
		struct poll_data p = { d };
		(void)write(pipefd[1], &p, sizeof(p));
		LOG("Distance: %f cms\n", d);
		LOG("Set!\n");
	} else {
		LOG("Not connected. Not set!\n");
	}
// To be part of the if() block of ioctl check.
// To be part of setInitialState().

// To be part of readEvents().
// To be part of event type proximity.
                    float d = // Save distance here.

		    LOG("Proximity event! Distance : %f cms\n", d);

		    if (connected) {
			struct poll_data p = { d };
			(void)write(pipefd[1], &p, sizeof(p));
			LOG("Distance: %f cms\n", d);
		    }	
// To be part of event type proximity.



