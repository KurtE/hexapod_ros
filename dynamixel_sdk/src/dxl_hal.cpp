/**
Modification of the HAL of the Dynamixel SDK to be used with USB2AX.

Nicolas Saugnier
*/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include "dxl_hal.h"

int gSocket_fd  = -1;
long    glStartTime = 0;
float   gfRcvWaitTime   = 0.0f;
float   gfByteTransTime = 0.0f;
int 	g_use_tcdrain = 0;	// assume we don't need tcdrain. 
// forward reference
extern int dxl_hal_map_baud(unsigned long baud);

int dxl_hal_open(const char* dev_name, unsigned long baud)
{
	struct termios newtio;
	struct serial_struct serinfo;
	char default_dev_name[] = "/dev/ttyACM0";

	// Build in support to explit device - /dev/ttyDXL
	dxl_hal_close();    // Make sure any previous handle is closed
	
	// if a device name is not passed in, we will default back to the original code which 
	// assummed /dev/ttyACM0
	if (!dev_name[0]) 
	{
		dev_name = default_dev_name;
	}

	if((gSocket_fd = open(dev_name, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}

	// We have an open file now see if it is FTDI so we know if tcdrain will help.
	char szProcFD[20];
	char szPath[30];
	int ich;
	int mapped_baud_value;

	sprintf(szProcFD, "/proc/self/fd/%d", gSocket_fd);
	ich = readlink(szProcFD, szPath, sizeof(szPath));
		
	// Hack look for /dev/ttyUSB... actuall only look at USB    
	if ((ich > 0) && (szPath[8]=='U') && (szPath[9]=='S')&& (szPath[10]=='B'))    
		g_use_tcdrain = 1;		// FTDI use drain...
	else    
		g_use_tcdrain = 0;		// Others ACM S2.. Don't appear to.

	
	gfByteTransTime = (float)((1000.0f / (float)baud) * 12.0f);
	
	memset(&newtio, 0, sizeof(newtio));
	mapped_baud_value = dxl_hal_map_baud(baud);

	newtio.c_cflag 		= CS8|CLOCAL|CREAD;
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// time-out ? (TIME * 0.1?) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN ? read ? return ?? ?? ?? ?? ??
	cfsetospeed(&newtio, mapped_baud_value);
	cfsetispeed(&newtio, mapped_baud_value);

	tcflush(gSocket_fd, TCIFLUSH);
	if (tcsetattr(gSocket_fd, TCSANOW, &newtio) < 0) {
		printf("tcsetattr %lu failed  %d\n\r", baud, errno);
	}
	return 1;

DXL_HAL_OPEN_ERROR:
	dxl_hal_close();
	return 0;
}

// Helper function to map Baud rates to the fixed baud rates of the termios system
// Should probably be some system function that does this?
int dxl_hal_map_baud(unsigned long baud)
{
	switch(baud)
	{
	case 1000000 : return(B1000000);
	case 2000000 : return(B2000000);
	case    9600 : return(B9600);
	case   19200 : return(B19200);
	case   38400 : return(B38400);
	case   57600 : return(B57600);
	case  115200 : return(B115200);
	case  230400 : return(B230400);
	case  460800 : return(B460800);
	case  500000 : return(B500000);
	case  576000 : return(B576000);
	case  921600 : return(B921600);
	case 1152000 : return(B1152000);
	case 1500000 : return(B1500000);
	case 2500000 : return(B2500000);
	case 3000000 : return(B3000000);
	case 3500000 : return(B3500000);
	case 4000000 : return(B4000000);
	default      : 
		printf("invalid baudrate\n");
	   return(B1000000);
	}
}

void dxl_hal_close()
{
	if(gSocket_fd != -1)
		close(gSocket_fd);
	gSocket_fd = -1;
}

int dxl_hal_set_baud( float baudrate )
{
//	struct serial_struct serinfo;
	
	if(gSocket_fd == -1)
		return 0;

	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
	return 1;
}

void dxl_hal_clear(void)
{
	tcflush(gSocket_fd, TCIFLUSH);
}

void dxl_hal_flush(void)
{
	// Some controllers like FTDI are helped others hurt by call to drain.
	if (g_use_tcdrain)
		tcdrain(gSocket_fd);
}
	
int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	return write(gSocket_fd, pPacket, numPacket);
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	memset(pPacket, 0, numPacket);
	return read(gSocket_fd, pPacket, numPacket);
}

static inline long myclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void dxl_hal_set_timeout( int NumRcvByte )
{
	glStartTime = myclock();
	gfRcvWaitTime = (float)(gfByteTransTime*(float)NumRcvByte + 5.0f);
}

int dxl_hal_timeout(void)
{
	long time;
	
	time = myclock() - glStartTime;
	
	if(time > gfRcvWaitTime)
		return 1;
	else if(time < 0)
		glStartTime = myclock();
		
	return 0;
}
