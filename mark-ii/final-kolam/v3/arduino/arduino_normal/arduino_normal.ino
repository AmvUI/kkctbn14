#include <Servo.h>


// ################################ DECLARE VARIABLE HERE ############################################## //
Servo esc1;
Servo esc2;
Servo s1, s2, s3, s4;
int esc_1 = 2;
int esc_2 = 3;
int leftMotor;
int rightMotor;
int servoKiriM   = 90;
int servoKiriB   = 80;
int servoKananM  = 180;
int servoKananB  = 10;
int input,num;
int buff[30];
int numTotal[5];
int j=-1;
int startFlag = 0;
int stopFlag = 0;
int sendLM,sendRM,sendSKiM,sendSKiB,sendSKaM,sendSKaB;

// ################################ DECLARE VARIABLE END ############################################### //
  int calc()
{
    int num=0,x=0;
 
    for(x;x<=j;x++)
          num=num+(buff[x]-48)*pow(10,j-x);           
    
    return num;
}
  
void setup(){
  Serial.begin(57600);
  esc1.attach(esc_1);
  esc2.attach(esc_2);
  s1.attach(4);
  s2.attach(5);
  s3.attach(6);
  s4.attach(7);
}

void loop(){
  if(Serial.available()>0)
    {
        
        input=Serial.read();
        if(input==',')
        {
            num=calc() +1;
            j=-1;
            //karena num=calc() +1; semua yang hanya berdigit 2 atau 1 jadi ditambah 1
            if (num == 8) startFlag = 1;
            if (num == 9) stopFlag = 1;
            if (num < 30000 && num >= 10000) numTotal[num/10000] = num - ((num/10000)*10000);
            if (num < 40 && num >= 30) numTotal[num/10] = (num - ((num/10)*10)) - 1;
            //numTotal[3] = 0 ialah maju
            //numTotal[3] = 1 ialah Rem Kiri
            //numTotal[3] = 2 ialah Rem Kanan
            //numTotal[3] = 3 ialah Rem Kiri Kanan
            //numTotal[3] = 4 ialah Belok Kiri
            //numTotal[3] = 5 ialah Belok Kanan
            
            
        }
        else
        {
            j++;
            buff[j]=input;
        }
        
    }
    
    if ( startFlag == 1 && stopFlag == 1){
      leftMotor  =    numTotal[1];
      rightMotor =    numTotal[2];
      if( numTotal[3] == 0){  //numTotal[3] = 0 ialah maju
        servoKiriM   = 90;
        servoKiriB   = 80;
        servoKananM  = 180;
        servoKananB  = 10;
      }
      
      if( numTotal[3] == 1){ //numTotal[3] = 1 ialah Rem Kiri
        servoKiriM   = 20;
        servoKiriB   = 80;
        servoKananM  = 180;
        servoKananB  = 10;
      }
  
      if( numTotal[3] == 2){ //numTotal[3] = 2 ialah Rem Kanan
        servoKiriM   = 90;
        servoKiriB   = 80;
        servoKananM  = 120;
        servoKananB  = 10;
      }
  
      if( numTotal[3] == 3){ //numTotal[3] = 3 ialah Rem Kiri Kanan
        servoKiriM   = 20;
        servoKiriB   = 80;
        servoKananM  = 120;
        servoKananB  = 10;    
      }
  
      if( numTotal[3] == 4){ //numTotal[3] = 4 ialah Belok Kiri
        servoKiriM   = 20;
        servoKiriB   = 40;
        servoKananM  = 180;
        servoKananB  = 30; 
      }
    
      if( numTotal[3] == 5){ //numTotal[3] = 5 ialah Belok Kanan
        servoKiriM   = 90;
        servoKiriB   = 40;
        servoKananM  = 120;
        servoKananB  = 30;  
      }
   
      startFlag = 0;
      stopFlag  = 0;
    }
    
 
    Serial.print( startFlag );
    Serial.print("  ");
    sendLM = leftMotor;
    Serial.print( sendLM);
    
    Serial.print("  ");
    sendRM = rightMotor;
    Serial.print( sendRM );
    
    Serial.print("  ");
    sendSKiM = servoKiriM;
    Serial.print( sendSKiM );
    
    Serial.print("  ");
    sendSKiB = servoKiriB;
    Serial.print( sendSKiB );
    
    Serial.print("  ");
    sendSKaM = servoKananM;
    Serial.print( sendSKaM );
    
    Serial.print("  ");
    sendSKaB = servoKananB;
    Serial.print( sendSKaB );
    
    Serial.print("  ");
    Serial.print( numTotal[3] );
    
    Serial.print("\n");
    
  esc1.writeMicroseconds(leftMotor);
  esc2.writeMicroseconds(rightMotor);
  s1.write(servoKiriM);
  s2.write(servoKiriB);
  s3.write(servoKananM);
  s4.write(servoKananB);

  }
 
