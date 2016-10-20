#include <Servo.h>


// ################################ DECLARE VARIABLE HERE ############################################## //
Servo esc1;
Servo esc2;
Servo kiMundur;
Servo kiBelok;
Servo kaMundur;
Servo kaBelok;

int esc_1 = 2;
int esc_2 = 3;
int kim = 4 ;
int kib = 5 ;
int kam = 6 ;
int kab = 7 ;
int leftMotor;
int rightMotor;
int kiriM,kiriB,kananM, kananB;
int input,num;
int buff[120];
int numTotal[8];
int j=-1;
int startFlag = 0;
int stopFlag = 0;

// ################################ DECLARE VARIABLE END ############################################### //
  int calc()
{
    int num=0,x=0;
 
    for(x;x<=j;x++)
          num=num+(buff[x]-48)*pow(10,j-x);           
    
    return num;
}
 
    
  
void setup(){
  Serial.begin(115200);
  esc1.attach(esc_1);
  esc2.attach(esc_2);
  kiMundur.attach(kim);
  kiBelok.attach(kib);
  kaMundur.attach(kam);
  kaBelok.attach(kab);
}

void loop(){
  if(Serial.available()>0)
   
    {
        
        input=Serial.read();
        if(input==',')
        {
            num=calc() +1;
            j=-1;
            if(num <= 30000 && num >= 10000)numTotal[num/10000] = num - ((num/10000)*10000);
            else if(num <= 7000 && num >= 3000 )numTotal[num/1000] = num - ((num/1000)*1000);
        }
        else
        {
            j++;
            buff[j]=input;
        }
        
    }
    
    
    leftMotor  =  numTotal[1];
    rightMotor =  numTotal[2];
    kiriM =  numTotal[3];
    kiriB =  numTotal[4];
    kananM =  numTotal[5];
    kananB =  numTotal[6];
    
    Serial.print( leftMotor);
    
    Serial.print("  ");
    Serial.print( rightMotor );
    
    Serial.print("  ");
    Serial.print( kiriM );
    
    Serial.print("  ");
    Serial.print( kiriB );
    
    Serial.print("  ");
    Serial.print( kananM );
    
    Serial.print("  ");
    Serial.print( kananB );
    
    Serial.print("  \n");
    
  esc1.writeMicroseconds(leftMotor);
  esc2.writeMicroseconds(rightMotor);
  kiMundur.write(kiriM);
  kiBelok.write(kiriB);
  kaMundur.write(kananM);
  kaBelok.write(kananB);

  }
 

