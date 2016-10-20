#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
 
/* My Arduino is on /dev/ttyACM0 */
char *portname = "/dev/ttyACM0";
char buf[256];
int dataOut = 65;
 
int main(int argc, char *argv[])
{
 int fd;
 int dataInt; 
/* Open the file descriptor in non-blocking mode */
 fd = open(portname, O_RDWR | O_NOCTTY);
 
/* Set up the control structure */
 struct termios toptions;
 
 /* Get currently set options for the tty */
 tcgetattr(fd, &toptions);
 
/* Set custom options */
 
/* 9600 baud */
 cfsetispeed(&toptions, B9600);
 cfsetospeed(&toptions, B9600);
 /* 8 bits, no parity, no stop bits */
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 /* no hardware flow control */
 toptions.c_cflag &= ~CRTSCTS;
 /* enable receiver, ignore status lines */
 toptions.c_cflag |= CREAD | CLOCAL;
 /* disable input/output flow control, disable restart chars */
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 /* disable canonical input, disable echo,
 disable visually erase chars,
 disable terminal-generated signals */
 toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 /* disable output processing */
 toptions.c_oflag &= ~OPOST;
 
/* wait for 24 characters to come in before read returns */
//Aslinya 24 cuman diganti jadi 3 karna nanti di OpenCV cuman 3 bilangan. 
toptions.c_cc[VMIN] = 2;
 /* no minimum time to wait before read returns */
 toptions.c_cc[VTIME] = 0;
 
/* commit the options */
 tcsetattr(fd, TCSANOW, &toptions);
 
/* Wait for the Arduino to reset */
 usleep(1000*1000);
 /* Flush anything already in the serial buffer */
 tcflush(fd, TCIFLUSH);

 /* read up to 128 bytes from the fd */
 //Aslinya 128 biar bisa baca 128 dari apapun yang dibuffer
 //tapi diubah biar tetep 3 bilangan aja 
 
 //Untuk ngubah dari char ke int
 
write(fd, &dataOut, 2);


return 0;
}
