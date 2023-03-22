#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define APP_VERSION			"1.0.0"
#define MSEC_PER_DAY			(24 * 60 * 60 * 1000)
#define LOOP_DELAY_S			10
#define TIME_IDX_IN_RT_OUT_STRING 	22
#define ZEPHYR_UPTIME_CMD		"kernel uptime\n"

long long timeInMilliseconds(void)
{
	struct timeval tv;

	gettimeofday(&tv,NULL);
	return (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
}

int main(int argc, char *argv[])
{
	printf("RT time difference checker\n");
	printf("Version: %s\n", APP_VERSION);
	printf("Enter Ctrl+c to exit\n");

	if (argc != 2) {
		printf("Invalid argument, usage:\n");
		printf("\t%s /dev/ttyACM0\n", argv[0]);
		return EINVAL;
	}

	int serial = open(argv[1], O_RDWR);

	if(serial < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return ENODEV;
	}

	struct termios tty = {};

	if(tcgetattr(serial, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return EIO;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 1;    // Wait for up to 0.1s
	tty.c_cc[VMIN] = 100;

	// Set in/out baud rate to be 115200
	cfsetspeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return EIO;
	}

	char read_buf[100];
	bool synchronized = false;
	long long start_rt_time_ms, start_linux_time_ms, current_linux_time_ms;

	do {
		if (synchronized) {
			sleep(LOOP_DELAY_S);
		}

		write(serial, ZEPHYR_UPTIME_CMD, sizeof(ZEPHYR_UPTIME_CMD) - 1);

		if (!synchronized) {
			start_linux_time_ms = timeInMilliseconds();
		} else {
			current_linux_time_ms = timeInMilliseconds();
		}

		memset(read_buf, '\0', sizeof(read_buf));

		ssize_t read_bytes = read(serial, read_buf, sizeof(read_buf));
		if (read_bytes < 0) {
			printf("Error reading: %s\n", strerror(errno));
			break;
		}

		char *strtoll_endptr;
		long long current_rt_time_ms;

		current_rt_time_ms = strtoll(&read_buf[TIME_IDX_IN_RT_OUT_STRING], &strtoll_endptr, 10);

		if (errno != 0) {
			perror("strtoll");
			break;
		}

		if (strtoll_endptr == &read_buf[TIME_IDX_IN_RT_OUT_STRING]) {
			fprintf(stderr, "Failed to parse RT output\n");
			continue;
		}

		if (!synchronized) {
			printf("Update every %us\n", LOOP_DELAY_S);
			printf("Time synchronized: RT %lld ms, Linux %lld ms\n",
			       current_rt_time_ms, start_linux_time_ms);
			start_rt_time_ms = current_rt_time_ms;
			synchronized = true;
			continue;
		}

		long long time_diff_ms =
			(current_rt_time_ms - start_rt_time_ms) - (current_linux_time_ms - start_linux_time_ms);
		long long time_diff_day_s =
			((time_diff_ms * MSEC_PER_DAY) / (current_linux_time_ms - start_linux_time_ms)) / 1000;
		printf("Time difference is %lld ms, %lld s/day\n", time_diff_ms, time_diff_day_s);
	} while(true);

	close(serial);

	return 0;
}
