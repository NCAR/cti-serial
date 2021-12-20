#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>


#if defined(Linux)

#include <sys/ioctl.h>

/* Change PATCHED-KERNEL below to the name of your patched kernel source directory */
#include "cti485.h"

#define MAXSIG		_NSIG
#define RTSFLOW		CRTSCTS
#define CTSFLOW		CRTSCTS

#elif defined(SunOS)

#include <sys/cti/blueheat.h>

#endif

#define DEV_DEFAULT	"/dev/ttyCTI7"
#define BAUD_DEFAULT	"9600"

int main(int argc, char **argv) {
	char *this = *argv;
	char *devname = DEV_DEFAULT;
	int dev;
	int baud;

	for (argc--, argv++; argc; argc--, argv++) {
		if (!strcmp(*argv, "-d")) {
			argc--; argv++;
			if (!argc) break;
			devname = *argv;
		} else if (!strcmp("-b", *argv)) {
			argc--; argv++;
			if (!argc) break;
			baud = atoi(*argv);
		} else if (!strcmp(*argv, "-h") || !strcmp(*argv, "-?")) {
			printf("\n");
			printf("%s [-d dev] [-b baud]\n", this);
			printf("\n");
			printf("-d dev\t\tdevice to use [" DEV_DEFAULT "]\n");
			printf("-b baud\t\tbaud rate [" BAUD_DEFAULT "]\n");
			return 0;
		} else {
			printf("%s: Error: Unrecognized option `%s`\n", this,
									*argv);
			return 1;
		}
	}

	printf("%s on %s\n", this, devname);
	dev = open(devname, O_RDWR | O_NONBLOCK);
	if (dev < 0) {
		printf("%s: Error: Couldn't open `%s` \n", this, devname);
		perror(this);
		return 1;
	}
	ioctl(dev, TIOCSERCTIBAUDSET, &baud);
	close(dev);

	return 0;
}
