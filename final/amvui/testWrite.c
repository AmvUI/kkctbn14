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
void serialRead(int dataRead){
read(fd, bufferRead, dataRead);
intRead = atoi(bufferRead);
 }

//serialWrite(Integer yang ingin dikirim, Besar Bilangan Integernya)
//Contoh : serialWrite(123,3);}

void serialWrite(int dataOut1,int dataOut2,int dataOut3,int dataOut4,int dataOut5,int dataOut6, int dataWrite){
sprintf(bufferWrite, "%d,%d,%d,%d,%d,%d,%d,%d,",8888,(dataOut1)+10000,(dataOut2)+20000,(dataOut3)+3000,(dataOut4)+4000,(dataOut5)+5000,(dataOut6)+6000,9999);
write(fd, bufferWrite, dataWrite);
}


void maju(){
  intscan3 = 0; //kiriMudur
  intscan4 = 80; //kiriBelok 
  intscan5 = 180; //kananMundur 
  intscan6 = 10; //kananBelok 
}
void rem(){
  intscan3 = 90; //  0 maju ; 90 kiri mundur
  intscan4 = 80; // < 100; 80; > 30 kiri belok
  intscan5 = 120; // 180 maju ; 120 mundur
  intscan6 = 10; //  <45 ; 25 ; 0>
}
void belokKiri(){
  intscan3 = 90; //  < 100; 60; > 30 kiri belok
  intscan4 = 100; // 5 maju ; 70 kiri mundur
  intscan5 = 180; // 120 mundur ; 180 kanan maju
  intscan6 = 30; //  <45 ; 25 ; 0>
}
void belokKanan(){

  intscan3 = 90; //  < 100; 60; > 30 kiri belok
  intscan4 = 100; // 5 maju ; 70 kiri mundur
  intscan5 = 120; // 120 mundur ; 180 kanan maju
  intscan6 = 30; //  <45 ; 25 ; 0>
}



int main(){

	maju();
	serialStart("/dev/ttyACM0", B57600, 42);
	    while(1) {
	scanf("%d %d %d", &intscan1,&intscan2,&mode);

 	if(mode ==0){maju();}
        if(mode ==1){rem();}
	if(mode == 2){belokKiri();}
	if(mode == 3){belokKanan();}
	if(mode == 4){
        maju();
	serialWrite(intscan1, intscan2, intscan3, intscan4, intscan5, intscan6 ,42);
	usleep(8000000);
	intscan1 = 900;
	intscan2 = 900;
	rem();
	serialWrite(intscan1, intscan2, intscan3, intscan4, intscan5, intscan6 ,42);
	usleep(1000000);
	intscan1 = 900;
	intscan2 = 900;
	belokKanan();
	serialWrite(intscan1, intscan2, intscan3, intscan4, intscan5, intscan6 ,42);
	usleep(5000000);
	intscan1 =1300;
	intscan2 = 1400;
	maju();
	serialWrite(intscan1, intscan2, intscan3, intscan4, intscan5, intscan6 ,42);
	usleep(4000000);
	}


	serialWrite(intscan1, intscan2, intscan3, intscan4, intscan5, intscan6 ,42);

	printf("Throttle : %d %d %d %d %d %d \n", intscan1, intscan2,intscan3, intscan4, intscan5, intscan6 );
	}
	return 0;
	}








