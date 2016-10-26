#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

/* My Arduino is on /dev/ttyACM0 */
char 		bufferRead[256];
char 		bufferWrite[42];
int 		intRead;
int 		fd;
int			motorKiri = 0;
int 		motorKanan = 0;
int 		kiriMundur 	= 0; //  0 maju ; 90 mundur
int 		kiriBelok 	= 80; // < 100; 60; > 30 
int			kananMundur 	= 180; // 180 maju ; 120 mundur
int			kananBelok 	= 10; //  <30 ; 10 ; 0>
int 		mode = 0 ;
const char* portName = "/dev/ttyACM0";

//serialStart(Lokasi Port, Baud Rate-nya, Besar Data yang ingin diRead untuk serialRead)
void serialStart(const char* portname, speed_t baud, int data)
{
 fd = open(portname, O_RDWR | O_NOCTTY);
 struct termios toptions;
 tcgetattr(fd, &toptions);
 cfsetispeed(&toptions, baud);
 cfsetospeed(&toptions, baud);
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 toptions.c_cflag &= ~CRTSCTS;
 toptions.c_cflag |= CREAD | CLOCAL;
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 toptions.c_oflag &= ~OPOST;
 toptions.c_cc[VMIN] = data;
 toptions.c_cc[VTIME] = 0;
 tcsetattr(fd, TCSANOW, &toptions);
 usleep(1000*1000);
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
  kiriMundur 	= 90; //  0 maju ; 90 mundur
  kiriBelok 	= 80; // < 100; 60; > 30 
  kananMundur 	= 180; // 180 maju ; 120 mundur
  kananBelok 	= 10; //  <30 ; 10 ; 0>
}
void rem(){
  kiriMundur 	= 20; //  5 maju ; 70 kiri mundur
  kiriBelok 	= 80; // < 100; 60; > 30 kiri belok
  kananMundur 	= 120; // 120 mundur ; 180 kanan maju
  kananBelok 	= 10; //  <45 ; 25 ; 0>
    }

void belokKiri(){
  kiriMundur 	= 0; //  < 100; 60; > 30 kiri belok
  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
  kananMundur 	= 180; // 120 mundur ; 180 kanan maju
  kananBelok 	= 30; //  <45 ; 25 ; 0>

}

void belokKanan(){
  kiriMundur 	= 90; //  < 100; 60; > 30 kiri belok
  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
  kananMundur 	= 130; // 120 mundur ; 180 kanan maju
  kananBelok 	= 30; //  <45 ; 25 ; 0>
}

int main(){

	maju();
	serialStart(portName, B57600, 42);
	    while(1) {
	scanf("%d %d %d", &motorKiri,&motorKanan,&mode);

 	if(mode ==0){maju();}
        if(mode ==1){rem();}
	if(mode == 2){belokKiri();}
	if(mode == 3){belokKanan();}
	if(mode == 4){
        maju();
	serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);
	usleep(8000000);
	motorKiri = 900;
	motorKanan = 900;
	rem();
	serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);
	usleep(1000000);
	motorKiri = 900;
	motorKanan = 900;
	belokKanan();
	serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);
	usleep(5000000);
	motorKiri =1300;
	motorKanan = 1400;
	maju();
	serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);
	usleep(4000000);
	}


	serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);

	printf("Throttle : %d %d %d %d %d %d \n", motorKiri, motorKanan,kiriMundur, kiriBelok, kananMundur, kananBelok );
	}
	return 0;
	}









