/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   SensorEmulationClientServer.c is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   SensorEmulationClientServer.c is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * SensorEmulationClientServer.c
 *
 * Author: Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 * Date : Sep 30th, 2013 - Nov 10, 2013
 *
 * Work done as part of my MS in CS thesis at Columbia University, NYC.
 *
 * Working:
 *
 * When DEVICE_READINGS is enabled.
 * 
 * Servers written as part of real device's sensors' HAL module
 * send the device's sensor readings in a pattern to any
 * connected clients over the network using socket communication. And
 * that pattern is pre-determined.
 *
 * This very code implements clients to a real Android device to get a
 * real android device's sensors readings over the network via
 * socket communication. These clients send these readings to dummy
 * forever-sleep servers at certain ports. This port is mapped on to the
 * guest(Android-x86)'s ports on Qemu. So, whatever the clients to the real
 * device write to the dummy forver-sleep servers, they all end up
 * in the guest.
 *
 * Inside the guest, emulator servers inside the sensors HAL
 * module receive these readings from the mapped ports. These servers
 * write onto the sensor event data that's polled periodically for each
 * of the sensor as a sensor event. Thus, a sensor-based app running
 * inside the guest gets a real device readings from the HAL module.
 *
 * When REMOTE_SERVER_READINGS is enabled.
 *
 * Except for the source of readings, which is a remote server in
 * this case, everything else in terms of working is same as it's in
 * the case of DEVICE_READINGS.
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>

#define DEBUG

// #define DEVICE_READINGS
// #define REMOTE_SERVER_READINGS

#ifdef DEBUG
static FILE *readings_fp;
#define INIT_LOG_READING do {\
				readings_fp = fopen("./ubuntu_readings", "w");\
			} while(0)
#ifdef DEVICE_READINGS

#define LOG_READING (void)(readings_fp && fprintf(readings_fp, "%s\n", dev_readings) && fflush(readings_fp))

#elif defined REMOTE_SERVER_READINGS

#define LOG_READING (void)(readings_fp && fprintf(readings_fp, "%s\n", rs_readings) && fflush(readings_fp))

#endif

#define LOG(...) (void)(printf("%s %d: ", __func__, __LINE__) && printf(__VA_ARGS__) && fflush(stdout))
#define LOG1(...) (void)(printf("%s %d: ", __func__, __LINE__) && printf(__VA_ARGS__) && fflush(stdout))
#define LOG_THREAD(...) (void)(printf("[%s] ", sensors_name[n]), LOG(__VA_ARGS__))
#define LOG1_THREAD(...) (void)(printf("[%s] ", sensors_name[n]), LOG1(__VA_ARGS__))
#define LOG_DUMMY_S_THREAD(...) (void)(printf("[%s] ", dummy_server_name[n]), LOG(__VA_ARGS__)) 
#define ERR(...) (void)(fprintf(stderr, "%s %d: ERROR - ", __func__, __LINE__) && fprintf(stderr, __VA_ARGS__) && fflush(stderr))
#define ERR1(...) (void)(fprintf(stderr, "%s %d: ERROR - ", __func__, __LINE__) && fprintf(stderr, __VA_ARGS__) && fflush(stderr))
#define ERR_THREAD(...) (void)(fprintf(stderr, "[%s] ", sensors_name[n]), ERR(__VA_ARGS__))
#define ERR1_THREAD(...) (void)(fprintf(stderr, "[%s] ", sensors_name[n]), ERR1(__VA_ARGS__))
#define ERR_DUMMY_S_THREAD(...) (void)(fprintf(stderr, "[%s] ", dummy_server_name[n]), ERR(__VA_ARGS__)) 

#else

#define LOG(...)
#define LOG1(...)
#define LOG_THREAD(...)
#define LOG1_THREAD(...)
#define ERR(...)
#define ERR1(...)
#define ERR_THREAD(...)
#define ERR1_THREAD(...)
#define INIT_LOG_READING
#define LOG_READING
#define ERR_DUMMY_S_THREAD(...)

#endif

#define LOCALHOST_IP "127.0.0.1"
#define BASE_PORT 5000
#define BASE_PORT_EMULATOR 5010

#ifdef DEVICE_READINGS
#define DEVICE_IP_PORT_CONF_FILE "./dev_ip_port.conf";
#elif defined REMOTE_SERVER_READINGS
#define REMOTE_SERVER_IP_PORT_CONF_FILE "./remote_server_ip_port.conf";
#endif

#define READINGS_BUF_SIZE (100) /* 3 readings. */
#define ACCEL_READINGS_BUF_SIZE (50) /* 3 readings. */
#define GYRO_READINGS_BUF_SIZE (50) /* 3 readings. */

#define NUM_SENSORS 10

static const char *sensors_name[NUM_SENSORS] = {
							"Accelerometer",
							"Magnetic",
							"Light",
							"Proximity",
							"Gyroscope",
							"Orientation",
							"CorrectedGyroscope",
							"Gravity",
							"LinearAcceleration",
							"RotationVector",
						};

static const char *dummy_server_name[NUM_SENSORS] = {
							"DummyServer-Accelerometer",
							"DummyServer-Magnetic",
							"DummyServer-Light",
							"DummyServer-Proximity",
							"DummyServer-Gyroscope",
							"DummyServer-Orientation",
							"DummyServer-CorrectedGyroscope",
							"DummyServer-Gravity",
							"DummyServer-LinearAcceleration",
							"DummyServer-RotationVector",
						};

enum sensors {
		EAccel = 0,
		EMagnet = 1,
		ELight = 2,
		EProx = 3,
		EGyro = 4,
		EOrient = 5,
		ECorrected = 6,
		EGravity = 7,
		ELinear = 8,
		ERotation = 9,
	};

static void cleanup(void);
static void cleanup_thread(int i);

static void sigsegv_handler(int arg)
{
	LOG("ATTENTION: **SIGSEGV** Exiting . . .\n");
	cleanup();
	exit(0);
}

static void sigabrt_handler(int arg)
{
	LOG("ATTENTION: **SIGABRT** Exiting . . .\n");
	cleanup();
	exit(0);
}

static int dummy_server_listenfd[NUM_SENSORS];
static int dummy_server_connfd[NUM_SENSORS];

struct dummy_server_data{
	int port;
	int num;
};

pthread_t dummy_server_pth[NUM_SENSORS];

static void *dummy_server(void *args)
{
	struct dummy_server_data *d = args;
	int port = d->port;
	int n = d->num;

	LOG_DUMMY_S_THREAD("** Dummy server for %s on behalf of the emulator server - Started! **\n", sensors_name[n]);
	LOG_DUMMY_S_THREAD("** Port : %d\n", port);
	LOG_DUMMY_S_THREAD("** Server number : %d\n", n);
	free(d);
	d = NULL;
	
	struct sockaddr_in serv_addr = { 0 };

	LOG_DUMMY_S_THREAD("Opening socket . . .\n");
	dummy_server_listenfd[n] = socket(AF_INET, SOCK_STREAM, 0);
	if (dummy_server_listenfd[n] == -1) {
		ERR_DUMMY_S_THREAD("socket - Opening failed - %s\n", strerror(errno));
		goto done;
	}
	LOG_DUMMY_S_THREAD("Opened!\n");
	
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_DUMMY_S_THREAD("Binding socket . . .\n");
	bool bound = bind(dummy_server_listenfd[n], (struct sockaddr*)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound && errno != EADDRINUSE) { // Due to port mapping with Qemu, this bind
						// may return EADDRINUSE. We just have to ignore it!
		ERR_DUMMY_S_THREAD("bind - Failed - %s %d\n", strerror(errno), errno);
		goto done;
	}
	LOG_DUMMY_S_THREAD("Bound!\n");

	LOG_DUMMY_S_THREAD("Trying to listen . . .\n");
	bool listening = listen(dummy_server_listenfd[n], 10) != -1;
	if (!listening) {
		ERR_DUMMY_S_THREAD("listen - Failed - %s\n", strerror(errno));
		goto done;
	}
	LOG_DUMMY_S_THREAD("Listening!\n");

	LOG_DUMMY_S_THREAD("Accepting connection . . .\n");
	dummy_server_connfd[n] = accept(dummy_server_listenfd[n], (struct sockaddr*)NULL, NULL);
	if (dummy_server_connfd[n] == -1) {
		ERR_DUMMY_S_THREAD("accept - Failed - %s\n", strerror(errno));
		goto done;
	}
	LOG_DUMMY_S_THREAD("Accepted!\n");

	LOG_DUMMY_S_THREAD("Sleeping forever . . .\n");
	sem_t dummy_sem = { { 0 } };
	bool initzd = sem_init(&dummy_sem, 0, 0) != -1;
	if (!initzd) {
		ERR_DUMMY_S_THREAD("sem_init - Failed - %s\n", strerror(errno));
		goto done;
	}
	bool waited = sem_wait(&dummy_sem) != -1;
	if (!waited) {
		LOG_DUMMY_S_THREAD("Couldn't sleep forever!\n");
		LOG_DUMMY_S_THREAD("\n\n** CONTROL SHOULD NEVER REACH HERE!! **\n\n");
		LOG_DUMMY_S_THREAD("\n\n** HELP ME!! I'M LOST!\n");
		ERR_DUMMY_S_THREAD("sem_wait - Failed - %s\n", strerror(errno));
		goto done;
	}

done:
	cleanup_thread(n);

	LOG_DUMMY_S_THREAD("** Dummy server for %s on behalf of the emulator server - Terminated! **\n", sensors_name[n]);

	return NULL;
}


static int emu_sockfd[NUM_SENSORS];

#ifdef DEVICE_READINGS
static int client_to_dev_sockfd[NUM_SENSORS];

struct client_to_device_data {
	int dev_port;
	int emu_port;
	int num;
};

static void *client_to_device(void *arg)
{
	struct client_to_device_data *d = arg;
	int n = d->num;

	LOG1_THREAD("** Client for getting readings from a device - Started! **\n");

	char ip[sizeof("xxx:xxx:xxx:xxx")] = "0.0.0.0";
	int dev_port = d->dev_port;

	const char *dev_ip_port_file = DEVICE_IP_PORT_CONF_FILE;
	FILE *dev_ip_port_fp = fopen(dev_ip_port_file, "r");
	if (dev_ip_port_fp) {
		LOG1_THREAD("Reading device's server ip and port from %s\n", dev_ip_port_file);

		bool fine = fscanf(dev_ip_port_fp, "%s", ip) == 1;
		if (!fine) {
			ERR1_THREAD("Something probably wrong with device ip or port - fscanf - %s\n", strerror(errno));
			goto done;
		}
		fgetc(dev_ip_port_fp); // Skip space


		fclose(dev_ip_port_fp);
		dev_ip_port_fp = NULL;
	} else {
		ERR1_THREAD("Failed to read %s. fopen - %s\n", dev_ip_port_file, strerror(errno));
		goto done;
	}

	struct sockaddr_in serv_addr = { 0, };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(dev_port);
	
	LOG1_THREAD("Converting server on device ip . . .\n");
	LOG1_THREAD("Device port: %d\n", dev_port);
	int af = AF_INET;
	int convert_ret = inet_pton(af, ip, &serv_addr.sin_addr);
	bool invalid_ip_str = convert_ret == 0;
	bool invalid_af_family = convert_ret == -1;
	if (invalid_ip_str) {
		ERR1_THREAD("Invalid ip str - %s\n", ip);
		goto done;
	} else if(invalid_af_family) {
		ERR1_THREAD("Invalid af family - %d\n", af);
		goto done;
	}
	LOG1_THREAD("Given device ip(%s) converted . . .\n", ip);

	strcpy(ip, LOCALHOST_IP);
	int emu_port = d->emu_port;

	free(d);
	d = NULL;

	struct sockaddr_in client_addr = { 0, };
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(emu_port);
	
	LOG1_THREAD("Converting emu ip . . .\n");
	LOG1_THREAD("Emu port: %d\n", emu_port);
	convert_ret = inet_pton(af, ip, &client_addr.sin_addr);
	invalid_ip_str = convert_ret == 0;
	invalid_af_family = convert_ret == -1;
	if (invalid_ip_str) {
		ERR1_THREAD("Invalid ip str - %s\n", ip);
		goto done;
	} else if(invalid_af_family) {
		ERR1_THREAD("Invalid af family - %d\n", af);
		goto done;
	}
	LOG1_THREAD("Given emulator ip(%s) converted . . .\n", ip);
	
	while (1) {
		if (client_to_dev_sockfd[n] != -1) {
			LOG1_THREAD("Closing socket . . .\n");
			close(client_to_dev_sockfd[n]);
			client_to_dev_sockfd[n] = -1;
			LOG1_THREAD("Closed!\n");
		}

		LOG1_THREAD("Opening device socket . . .\n");
		client_to_dev_sockfd[n] = socket(AF_INET, SOCK_STREAM, 0);
		if (client_to_dev_sockfd[n] == -1) {
			ERR1_THREAD("socket - %s\n", strerror(errno));
			continue;
		}
		LOG1_THREAD("Device socket opened . . .\n");

		LOG1_THREAD("Connecting . . .\n");
		bool connected = connect(client_to_dev_sockfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
		if (!connected) {
			ERR1_THREAD("connect - %s\n", strerror(errno));
			sleep(1);
			continue;
		}
		LOG1_THREAD("Connected . . .\n");

		if (emu_sockfd[n] != -1) {
			LOG1_THREAD("Closing emu socket . . .\n");
			close(emu_sockfd[n]);
			emu_sockfd[n] = -1;
			LOG1_THREAD("Closed!\n");
		}

		LOG1_THREAD("Opening emu socket . . .\n");
		emu_sockfd[n] = socket(AF_INET, SOCK_STREAM, 0);
		if (emu_sockfd[n] == -1) {
			ERR1_THREAD("socket - %s\n", strerror(errno));
			continue;
		}
		LOG1_THREAD("Emu socket opened!\n");

		LOG1_THREAD("Connecting . . .\n");
		connected = connect(emu_sockfd[n], (struct sockaddr *)&client_addr, sizeof(client_addr)) != -1;
		if (!connected) {
			ERR1_THREAD("connect - %s\n", strerror(errno));
			continue;
		}
		LOG1_THREAD("Connected!\n");

		while (1) {
			size_t readings_size = n == EAccel ? ACCEL_READINGS_BUF_SIZE + 1 :
						n == EGyro ? GYRO_READINGS_BUF_SIZE + 1 : READINGS_BUF_SIZE + 1;
			char dev_readings[readings_size];
			memset(dev_readings, 0, sizeof(dev_readings));

			LOG1_THREAD("Reading . . .\n");
			ssize_t bytes_received = recvfrom(client_to_dev_sockfd[n], dev_readings, readings_size, MSG_WAITALL, NULL, 0);
			if (bytes_received == -1) {
				ERR1_THREAD("recvfrom - %s\n", strerror(errno));
				break;
			}
			LOG1_THREAD("%zd bytes read!\n", bytes_received);
			LOG1_THREAD("Device readings: %s\n", dev_readings);

			LOG_READING;

			if (!bytes_received) {
				LOG1_THREAD("Seems like connection to device server is lost. Will reconnect.\n");
				break;
			}

			if (bytes_received != readings_size) {
				LOG1_THREAD("Partial data. Ignoring\n");
				continue;
			}

			LOG1_THREAD("Sending to emulator via port redirection!\n");
			ssize_t bytes_sent = sendto(emu_sockfd[n], dev_readings, readings_size, 0, NULL, 0);
			if (bytes_sent == -1) {
				ERR1_THREAD("sendto - %s\n", strerror(errno));
				break;
			}
			LOG1_THREAD("%zd bytes wrote!\n", bytes_sent);

			usleep(1000);
		}
	}

done:
	if (client_to_dev_sockfd[n] != -1) {
		close(client_to_dev_sockfd[n]);
		client_to_dev_sockfd[n] = -1;
	}

	LOG1_THREAD("** Client for %s meant for getting readings from a device - Terminated! **\n", sensors_name[n]);

	cleanup_thread(n);

	return 0;
}

pthread_t client_to_dev_pth[NUM_SENSORS];

#elif defined REMOTE_SERVER_READINGS

static int client_to_rs_sockfd[NUM_SENSORS];

struct client_to_rs_data {
	int rs_port;
	int emu_port;
	int num; 
};

static void *client_to_remote_server(void *arg)
{
	struct client_to_rs_data *r = arg;

	char ip[sizeof("xxx:xxx:xxx:xxx")] = "0.0.0.0";
	int rs_port = r->rs_port;
	int n = r->num;

	LOG1_THREAD("** Client for %s meant for getting readings from a remote server - Started! **\n", sensors_name[n]);

	const char *rs_ip_port_file = REMOTE_SERVER_IP_PORT_CONF_FILE;
	FILE *rs_ip_port_fp = fopen(rs_ip_port_file, "r");
	if (rs_ip_port_fp) {
		LOG1_THREAD("Reading remote server ip and port from %s\n", rs_ip_port_file);

		bool fine = fscanf(rs_ip_port_fp, "%s", ip) == 1;
		if (!fine) {
			ERR1_THREAD("Something probably wrong with remote server ip or port - fscanf - %s\n", strerror(errno));
			goto done;
		}
		fgetc(rs_ip_port_fp); // Skip space

		fclose(rs_ip_port_fp);
		rs_ip_port_fp = NULL;
	} else {
		ERR1_THREAD("Failed to read %s. fopen - %s\n", rs_ip_port_file, strerror(errno));
		goto done;
	}

	struct sockaddr_in serv_addr = { 0, };
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(rs_port);
	
	LOG1_THREAD("Converting remote server ip . . .\n");
	LOG1_THREAD("Remote server port: %d\n", rs_port);
	int af = AF_INET;
	int convert_ret = inet_pton(af, ip, &serv_addr.sin_addr);
	bool invalid_ip_str = convert_ret == 0;
	bool invalid_af_family = convert_ret == -1;
	if (invalid_ip_str) {
		ERR1_THREAD("Invalid ip str - %s\n", ip);
		goto done;
	} else if(invalid_af_family) {
		ERR1_THREAD("Invalid af family - %d\n", af);
		goto done;
	}
	LOG1_THREAD("Given remote server ip(%s) converted . . .\n", ip);

	strcpy(ip, LOCALHOST_IP);
	int emu_port = r->emu_port;

	free(r);
	r = NULL;
	
	struct sockaddr_in client_addr = { 0, };
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(emu_port);
	
	LOG1_THREAD("Converting emu ip . . .\n");
	LOG1_THREAD("Emu port: %d\n", emu_port);
	convert_ret = inet_pton(af, ip, &client_addr.sin_addr);
	invalid_ip_str = convert_ret == 0;
	invalid_af_family = convert_ret == -1;
	if (invalid_ip_str) {
		ERR1_THREAD("Invalid ip str - %s\n", ip);
		goto done;
	} else if(invalid_af_family) {
		ERR1_THREAD("Invalid af family - %d\n", af);
		goto done;
	}
	LOG1_THREAD("Given emulator ip(%s) converted . . .\n", ip);
	
	while (1) {
		if (client_to_rs_sockfd[n] != -1) {
			LOG1_THREAD("Closing client to remote server socket . . .\n");
			close(client_to_rs_sockfd[n]);
			client_to_rs_sockfd[n] = -1;
			LOG1_THREAD("Closed!\n");
		}

		LOG1_THREAD("Opening remote server socket . . .\n");
		client_to_rs_sockfd[n] = socket(AF_INET, SOCK_STREAM, 0);
		if (client_to_rs_sockfd[n] == -1) {
			ERR1_THREAD("Socket - %s\n", strerror(errno));
			goto done;
		}
		LOG1_THREAD("Remote server socket opened . . .\n");

		LOG1_THREAD("Connecting . . .\n");
		bool connected = connect(client_to_rs_sockfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
		if (!connected) {
			ERR1_THREAD("Connect - %s\n", strerror(errno));
			sleep(1);
			continue;
		}
		LOG1_THREAD("Connected . . .\n");

		if (emu_sockfd[n] != -1) {
			LOG1_THREAD("Closing emu socket . . .\n");
			close(emu_sockfd[n]);
			emu_sockfd[n] = -1;
			LOG1_THREAD("Closed!\n");
		}

		LOG1_THREAD("Opening emu socket . . .\n");
		emu_sockfd[n] = socket(AF_INET, SOCK_STREAM, 0);
		if (emu_sockfd[n] == -1) {
			ERR1_THREAD("socket - %s\n", strerror(errno));
			goto done;
		}
		LOG1_THREAD("Emu socket opened!\n");

		LOG1_THREAD("Connecting . . .\n");
		connected = connect(emu_sockfd[n], (struct sockaddr *)&client_addr, sizeof(client_addr)) != -1;
		if (!connected) {
			ERR1_THREAD("connect - %s\n", strerror(errno));
			goto done;
		}
		LOG1_THREAD("Connected!\n");

		time_t seed = time(NULL);
		if (seed == -1) {
			ERR1_THREAD("time - Failed to generate a seed - %s\n", strerror(errno));
			// Not a crtical error. So, we continue!
		}
		srand(seed);

		while (1) {
			size_t readings_size = n == EAccel ? ACCEL_READINGS_BUF_SIZE + 1 :
						n == EGyro ? GYRO_READINGS_BUF_SIZE + 1 : READINGS_BUF_SIZE + 1;

			char rs_readings[readings_size];
			memset(rs_readings, 0, sizeof(rs_readings));

			LOG1_THREAD("Receiving . . .\n");
			ssize_t bytes_received = recvfrom(client_to_rs_sockfd[n], rs_readings, readings_size, MSG_WAITALL, NULL, 0);
			if (bytes_received == -1) {
				ERR1_THREAD("recvFrom - %s\n", strerror(errno));
				break;
			}
			LOG1_THREAD("%zd bytes received!\n", bytes_received);
			LOG1_THREAD("Remote server readings: %s\n", rs_readings);

			LOG_READING;

			if (!bytes_received) {
				LOG1_THREAD("Seems like connection to remote server is lost. Will reconnect.\n");
				break;
			}

			if (bytes_received != readings_size) {
				LOG1_THREAD("Partial data. Ignoring\n");
				continue;
			}

			LOG1_THREAD("Sending to emulator via port redirection!\n");
			ssize_t bytes_sent = sendto(emu_sockfd[n], rs_readings, readings_size, 0, NULL, 0);
			if (bytes_sent == -1) {
				ERR1_THREAD("sendto - %s\n", strerror(errno));
				break;
			} else {
				LOG1_THREAD("%zd bytes wrote!\n", bytes_sent);
			}

			usleep(1000);
		}
	}

done:
	if (client_to_rs_sockfd[n] != -1) {
		close(client_to_rs_sockfd[n]);
		client_to_rs_sockfd[n] = -1;
	}

	LOG1_THREAD("** Client for %s meant for getting readings from a remote server - Terminated! **\n", sensors_name[n]);

	cleanup_thread(n);

	return 0;
}

pthread_t client_to_rs_pth[NUM_SENSORS];
#endif

static void cleanup_thread(int i)
{
#ifdef DEVICE_READINGS
	i = 0;
	if (client_to_dev_sockfd[i] != -1) {
		close(client_to_dev_sockfd[i]);
		client_to_dev_sockfd[i] = -1;
	}
#elif defined REMOTE_SERVER_READINGS
	if (client_to_rs_sockfd[i] != -1) {
		close(client_to_rs_sockfd[i]);
		client_to_rs_sockfd[i] = -1;
	}
#endif

	if (dummy_server_listenfd[i] != -1) {
		close(dummy_server_listenfd[i]);
		dummy_server_listenfd[i] = -1;
	}

	if (dummy_server_connfd[i] != -1) {
		close(dummy_server_connfd[i]);
		dummy_server_connfd[i] = -1;
	}

#ifdef DEVICE_READINGS
	if (client_to_dev_pth[i] != -1) {
		pthread_cancel(client_to_dev_pth[i]);
		client_to_dev_pth[i] = -1;
	}
#elif defined REMOTE_SERVER_READINGS
	if (client_to_rs_pth[i] != -1) {
		pthread_cancel(client_to_rs_pth[i]);
		client_to_rs_pth[i] = -1;
	}
#endif

	if (emu_sockfd[i] != -1) {
		close(emu_sockfd[i]);
		emu_sockfd[i] = -1;
	}

	if (dummy_server_pth[i] != -1) {
		pthread_cancel(dummy_server_pth[i]);
	}
}

static void cleanup(void)
{
	LOG("Cleaning up . . .\n");

	int i = 0;
	while (i < NUM_SENSORS) {
		cleanup_thread(i);
		i++;
	}

	LOG("Cleaned!\n");

	LOG("** SensorEmulationClientServer - Exited **\n");

	putchar('\n');
	exit(0);
}

static void init_fds_pth(void)
{
	int i = 0;
	while (i < NUM_SENSORS) {
		dummy_server_listenfd[i] = dummy_server_connfd[i] = -1;
#ifdef DEVICE_READINGS
		client_to_dev_sockfd[i] = -1;
		client_to_dev_pth[i] = -1;
#elif defined REMOTE_SERVER_READINGS
		client_to_rs_sockfd[i] = -1;
		client_to_rs_pth[i] = -1;
#endif
		emu_sockfd[i] = -1;

		dummy_server_pth[i] = -1;
		i++;
	}	
}

static void sigint_handler(int sig)
{
	LOG("** Interrupted - exiting.\n");
	cleanup();
	exit(0);
}

int main(void)
{
	(void)signal(SIGINT, sigint_handler);
	(void)signal(SIGSEGV, sigsegv_handler);
	(void)signal(SIGABRT, sigabrt_handler);

	LOG("** SensorEmulationClientServer - Started! **\n");

#ifdef DEVICE_READINGS
	LOG("DEVICE_READINGS!\n");
#elif defined REMOTE_SERVER_READINGS
	LOG("REMOTE_SERVER_READINGS!\n");
#else
	LOG("NOTE: Neither DEVICE_READINGS nor REMOTE_SERVER_READINGS!\n");
#endif

	INIT_LOG_READING;

	init_fds_pth();

	int i = 0;

	while (i < NUM_SENSORS) {
		struct dummy_server_data *d = malloc(sizeof(*d));
		if (!d) {
			ERR("malloc - %s\n", strerror(errno));
			goto done;
		}
		d->port = BASE_PORT + i;
		d->num = i;

		errno = pthread_create(&dummy_server_pth[i], NULL, dummy_server, d);
		bool created = !errno;
		if (!created) {
			ERR("pthread_create - Failed to create dummy server - %s\n", strerror(errno));
			goto done;
		}
		i++;
	}

#ifdef DEVICE_READINGS
	i = 0;
	while (i < NUM_SENSORS) {
		struct client_to_device_data *d = malloc(sizeof(*d));
		if (!d) {
			ERR("malloc - %s\n", strerror(errno));
			goto done;
		}
		d->dev_port = d->emu_port = BASE_PORT + i;
		d->num = i;

		errno = pthread_create(&client_to_dev_pth[i], NULL, client_to_device, d);

		bool created = !errno;
		if (!created) {
			ERR("pthread_create - Client to device failed - %s\n", strerror(errno));
			goto done;
		}
		i++;
	}
#elif defined REMOTE_SERVER_READINGS
	i = 0;
	while (i < NUM_SENSORS) {
		struct client_to_rs_data *r = malloc(sizeof(*r));
		if (!r) {
			ERR("malloc - %s\n", strerror(errno));
			goto done;
		}
		r->emu_port = BASE_PORT + i;
		r->rs_port = BASE_PORT_EMULATOR + i;
		r->num = i;

		errno = pthread_create(&client_to_rs_pth[i], NULL, client_to_remote_server, r);
		bool created = !errno;
		if (!created) {
			ERR("pthread_create - Client to remote server failed - %s\n", strerror(errno));
			goto done;
		}
		i++;
	}
#endif

#ifdef DEVICE_READINGS
	i = 0;
	while (i < NUM_SENSORS) {
		if (client_to_dev_pth[i] != -1) {
			bool joined = pthread_join(client_to_dev_pth[i], NULL) != -1;
			if (!joined) {
				ERR("pthread_join - Failed to wait for client to device - %s\n", strerror(errno));
				goto done;
			}
		}
		i++;
	}
#elif defined REMOTE_SERVER_READINGS
	i = 0;
	while (i < NUM_SENSORS) {
		if (client_to_rs_pth[i] != -1) {
			bool joined = pthread_join(client_to_rs_pth[i], NULL) != -1;
			if (!joined) {
				ERR("pthread_join - Failed to wait for client to remote server - %s\n", strerror(errno));
				goto done;
			}
		}
		i++;
	}
#endif

	i = 0;
	while (i < NUM_SENSORS) {
		if (dummy_server_pth[i] != -1) {
			bool joined = pthread_join(dummy_server_pth[i], NULL) != -1;
			if (!joined) {
				ERR("pthread_cancel - Faile to wait for dummy server - %s\n", strerror(errno));
			}
		}
		i++;
	}

	LOG("** CAUTION: SensorEmulation dummy server returned - UNEXPECTED! Exiting . . .\n");
done:

	cleanup();

	LOG("** SensorEmulationClientServer - Terminated! **\n");

	return 0;
}



