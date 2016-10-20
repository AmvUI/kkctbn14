#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
 
/* My Arduino is on /dev/ttyACM0 */
char bufferRead[256];
int intRead; 
int fd;

//serialStart(Lokasi Port, Baud Rate-nya, Besar Data yang ingin diRead untuk serialRead)
void serialStart(char *portname,speed_t baud, int data)
{
/* Open the file descriptor in non-blocking mode */
 fd = open(portname, O_RDWR | O_NOCTTY);
 
/* Set up the control structure */
 struct termios toptions;
 
 /* Get currently set options for the tty */
 tcgetattr(fd, &toptions);
 
/* Set custom options */
 
/* 9600 baud */
 cfsetispeed(&toptions, baud);
 cfsetospeed(&toptions, baud);
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
toptions.c_cc[VMIN] = data;
 /* no minimum time to wait before read returns */
 toptions.c_cc[VTIME] = 0;
 
/* commit the options */
 tcsetattr(fd, TCSANOW, &toptions);
 
/* Wait for the Arduino to reset */
 usleep(1000*1000);
 /* Flush anything already in the serial buffer */
 tcflush(fd, TCIFLUSH);

 }

//Data tersimpan di BufferRead, untuk data yang udah jadi integer bisa pakai intRead
//serialRead(Besar Data yang akan diread)
//Contoh Test : serialRead(3); printf("%d   %d\n", intRead, intRead*2 );
void serialRead(int dataRead){ 
read(fd, bufferRead, dataRead);
intRead = atoi(bufferRead);
 }
 
//Ada bit 0 diakhir setiap ngirim data. Buat filter di mikon untuk nge-abaikaan 0
//serialWrite(Integer yang ingin dikirim, Besar Bilangan Integernya)
//Contoh : serialWrite(123,3);}

void serialWrite(int dataOut, int dataWrite){
	write(fd, &dataOut, dataWrite);
}


int main()
{
serialStart("/dev/ttyACM0",B9600,3);
while(1) serialWrite(123,3);
return 0;
}
