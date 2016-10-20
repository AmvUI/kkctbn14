#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

/* My Arduino is on /dev/ttyACM0 */
char bufferRead[256];
char bufferWrite[42];
int intRead;
int fd;
int intscan1 = 0;
int intscan2 = 0;
int intscan3 = 5 ;
int intscan4 = 60;
int intscan5 = 180 ;
int intscan6 = 25;
int mode =0 ;


//serialStart(Lokasi Port, Baud Rate-nya, Besar Data yang ingin diRead untuk serialRead)
void serialStart(const char* portname, speed_t baud, int data)
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
int serialRead(int dataRead){
read(fd, bufferRead, dataRead);
intRead = atoi(bufferRead);

return 1;
 }

//serialWrite(Integer yang ingin dikirim, Besar Bilangan Integernya)
//Contoh : serialWrite(123,3);}

void serialWrite(int dataOut1,int dataOut2,int dataOut3,int dataOut4,int dataOut5,int dataOut6, int dataWrite){
sprintf(bufferWrite, "%d,%d,%d,%d,%d,%d,%d,%d,",8888,(dataOut1)+10000,(dataOut2)+20000,(dataOut3)+3000,(dataOut4)+4000,(dataOut5)+5000,(dataOut6)+6000,9999);
write(fd, bufferWrite, dataWrite);
}


void maju(){
  intscan3 = 5; //  5 maju ; 70 kiri mundur
  intscan4 = 75; // < 100; 60; > 30 kiri belok
  intscan5 = 180; // 120 mundur ; 180 kanan maju
  intscan6 = 20; //  <45 ; 25 ; 0>
}
void rem(){
  intscan3 = 70; //  5 maju ; 70 kiri mundur
  intscan4 = 75; // < 100; 60; > 30 kiri belok
  intscan5 = 120; // 120 mundur ; 180 kanan maju
  intscan6 = 20; //  <45 ; 25 ; 0>
}
void belokKiri(){
   intscan3 = 70; //  < 100; 60; > 30 kiri belok
  intscan4 = 30; // 5 maju ; 70 kiri mundur
  intscan5 = 180; // 120 mundur ; 180 kanan maju
  intscan6 = 45; //  <45 ; 25 ; 0>

}

void belokKanan(){
  intscan3 = 5; //  < 100; 60; > 30 kiri belok
  intscan4 = 30; // 5 maju ; 70 kiri mundur
  intscan5 = 120; // 120 mundur ; 180 kanan maju
  intscan6 = 45; //  <45 ; 25 ; 0>

}



int main(){


	serialStart("/dev/ttyACM1", B115200, 42);
	    while(1) {
 
	if (serialRead(42) == 1){
	printf("%s\n", bufferRead );
}
	}


	return 0;
	}









