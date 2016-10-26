#include <Servo.h>


// ################################ DECLARE VARIABLE HERE ############################################## //
Servo esc1;
Servo esc2;
Servo s1, s2, s3, s4;
int esc_1 = 2;
int esc_2 = 3;
int leftMotor;
int rightMotor;
int servoKiriM =0;
int servoKiriB =90;
int servoKananM = 100;
int servoKananB =10;
int input,num;
int buff[30];
int numTotal[10];
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
            if (num == 8888) startFlag = 1;
            if (num == 9999) stopFlag = 1;
            if (num < 30000 && num >= 10000) numTotal[num/10000] = num - ((num/10000)*10000);
            if (num < 7000 && num >= 3000) numTotal[num/1000] = num - ((num/1000)*1000);
            
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
    servoKiriM =    numTotal[3];
    servoKiriB =    numTotal[4];
    servoKananM =   numTotal[5];
    servoKananB =   numTotal[6];
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
    Serial.print( stopFlag );
    
    Serial.print("\n");
    
  esc1.writeMicroseconds(leftMotor);
  esc2.writeMicroseconds(rightMotor);
  s1.write(servoKiriM);
  s2.write(servoKiriB);
  s3.write(servoKananM);
  s4.write(servoKananB);

  }
 
