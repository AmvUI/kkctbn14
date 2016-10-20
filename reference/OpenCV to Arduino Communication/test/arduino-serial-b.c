#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

int main(){
	int sp=open("/dev/ttyACM0",O_RDWR);
	system("cat < /dev/ttyACM0");
	sleep(2);
	
	unsigned char c;
	for(c=0;c<255;c++) write(sp,&c,1);
	while(read(sp,&c,1)) printf("got %d\n",c);
}
