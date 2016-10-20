#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <ctype.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

struct termios stdin_orig;  // Structure to save parameters

void term_reset() {
        tcsetattr(STDIN_FILENO,TCSANOW,&stdin_orig);
        tcsetattr(STDIN_FILENO,TCSAFLUSH,&stdin_orig);
}

void term_nonblocking() {
        struct termios newt;
        tcgetattr(STDIN_FILENO, &stdin_orig);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // non-blocking
        newt = stdin_orig;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        atexit(term_reset);
}

int main(){
	term_nonblocking();
	int i,b;
	while (1 & b <= 0){
		b = getchar();
		
		
		printf("Test %d\n", i);
		i += 1;
		
		usleep(1000*1000);
		
		}
	
	
}
