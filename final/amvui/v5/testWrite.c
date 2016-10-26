#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

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
int			nozzleState	= 0;
char 		movementStat[20];
char		statBelokKiri[20] = "Belok Kiri";
char		statBelokKanan[20] = "Belok Kanan";
char		statMaju[20] = "Maju";
char		statRem[20] = "REM";
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

void serialWrite(int dataOut1,int dataOut2,int dataOut3, int dataWrite){
//Lihat diarduino, awalnya startFlag emang 8 dan stopFlag 9 namun karena
//ada bug di algoritma untuk ngubah asci jadi int == >  karena num=calc() +1; 
//semua yang hanya berdigit 2 atau 1 jadi ditambah 1
sprintf(bufferWrite, "%d,%d,%d,%d,%d,",7,(dataOut1)+10000,(dataOut2)+20000,(dataOut3)+30,8);
write(fd, bufferWrite, dataWrite);
}	

void maju(){
//  kiriMundur 	= 90; //  0 maju ; 90 mundur
//  kiriBelok 	= 80; // < 100; 60; > 30 
//  kananMundur 	= 180; // 180 maju ; 120 mundur
//  kananBelok 	= 10; //  <30 ; 10 ; 0>
nozzleState		=	0;

}
void rem(int nozzleSide){
//  kiriMundur 	= 20; //  5 maju ; 70 kiri mundur
//  kiriBelok 	= 80; // < 100; 60; > 30 kiri belok
//  kananMundur 	= 120; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 10; //  <45 ; 25 ; 0>
if (nozzleSide == 1){
	nozzleState		=	1;
	}

if (nozzleSide == 2){
	nozzleState		=	2;
	}

if (nozzleSide == 3){
	nozzleState		=	3;
	}
}

void belokKiri(){
//  kiriMundur 	= 20; //  < 100; 60; > 30 kiri belok
//  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
//  kananMundur 	= 180; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 30; //  <45 ; 25 ; 0>
	nozzleState		=	4;
}

void belokKanan(){
//  kiriMundur 	= 90; //  < 100; 60; > 30 kiri belok
//  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
//  kananMundur 	= 130; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 30; //  <45 ; 25 ; 0>
	nozzleState		=	5;
}

int main(){

	maju();
	serialStart(portName, B115200, 19);
	    while(1) {
	scanf("%d %d %d", &motorKiri,&motorKanan,&mode);

 	if(mode ==0){maju();strcpy(movementStat, statMaju);}
    if(mode ==1){rem(3);strcpy(movementStat, statRem);}
	if(mode == 2){belokKiri();strcpy(movementStat, statBelokKiri);}
	if(mode == 3){belokKanan();strcpy(movementStat, statBelokKanan);}


	serialWrite(motorKiri, motorKanan, nozzleState,19);

	printf("Throttle : %d %d %d %s\n", motorKiri, motorKanan,nozzleState, movementStat);
	}
	return 0;
	}









