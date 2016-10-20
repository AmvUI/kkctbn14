#include<stdio.h>
#include<time.h>

	int main()
	{
    		 int number; 
 
          FILE* file = fopen("/dev/ttyACM0", "r"); // read only  
         
          if (!file ) // equivalent to saying if ( in_file == NULL ) 
             {  
                printf("oops, file can't be read\n"); 
               } 
 
          // attempt to read the next line and store 
          // the value in the "number" variable 
          while (  1 )  
             { 
		fscanf(file, "%d", & number );
         
	if(number > 100)printf("We just read %d\n", number); 
             
	} 
            
	}
