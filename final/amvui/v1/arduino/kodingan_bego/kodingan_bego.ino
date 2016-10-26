#include <Servo.h>

Servo escl, escr;
Servo s1, s2, s3, s4;
// dihitung dari kiri ke kanan

 

void setup()
{
  Serial.begin(9600);
  
  escl.attach(2); // motor kiri
  escr.attach(3); // motor kanan 

  s1.attach(4); //  kiri mundur
  s2.attach(5); //  kiri belok
  s3.attach(6); //  kanan mundur
  s4.attach(7); //  kanan belok
}

void jalan(int esc_l, int esc_r, int ki_m, int ki_b, int ka_m, int ka_b)
{
  escl.writeMicroseconds(esc_l);
  escr.writeMicroseconds(esc_r);
  
  s1.write(ki_m);   // 0 maju ; 60 kiri mundur
  s2.write(ki_b); //  < ?130; 90; > 70 kiri belok
  
  s3.write(ka_m); // 120 mundur ; 180 kanan maju
  s4.write(ka_b); //  <? ; 10 ; 30 >

  
  Serial.print(esc_l);
  Serial.print(" ");
  Serial.print(esc_r);
  Serial.print(" ");
  Serial.print(ki_m);
  Serial.print(" ");
  Serial.print(ki_b);
  Serial.print(" ");
  Serial.print(ka_m);
  Serial.print(" ");
  Serial.print(ka_b);
  Serial.println(" ");

}

void maju(){
  jalan(1200, 1200, 90, 80, 180, 10);
}

void rem(){
  jalan(1000, 1000, 20, 80, 120, 10);
}

void belok_kiri(){
  jalan(1200, 1100, 0, 40, 180, 30);
}

void belok_kanan(){
  jalan(1100, 1200, 90, 40, 130, 30);
}

int mode = 2;

void loop()
{
/*

  // --- S kebalik Bego ---
  jalan(1200, 1200, 90, 80, 180, 10); //lurus
  delay(3000);
  jalan(1200, 850, 90, 80, 180, 10);  //kanan dikit
  delay(2000);
  jalan(1200, 700, 90, 70, 180, 10);  //kanan bgt
  delay(5000);
  
  jalan(700, 1200, 90, 90, 180, 30);  //kiri bgt
  delay(2000);
  
  jalan(1200, 1200, 90, 90, 180, 10); //lurus
  delay(3000);
  jalan(850, 1200, 0, 90, 180, 20);  //kiri dikit
  delay(2000);
  jalan(700, 1200, 0, 90, 180, 30);  //kiri bgt
  delay(5000);
  
  jalan(1200, 700, 0, 70, 180, 10);  //kanan bgt
  delay(2000);
  
  */  
// kalo mode = 1, kapal mode speed test. kalo mode =/ 1, kapal mode manuver
// masukin modenya manual ada di atas void (int mode)

  if(mode==1) {
  // --- speed test ---
  maju();
  delay(2000);
  jalan(1300, 1100, 90, 80, 180, 10);
  delay(6000);
  
  rem();
  delay(3000);
  
  belok_kanan();
  delay(6000);
  } 
  else
  {  
    
  // --- manuver ---
  maju();
  delay(7000);
  
  rem();
  delay(2000);
  
  belok_kanan();
  delay(4000);
  
  belok_kiri();
  delay(1500);
  
  maju();
  delay(5000);
  
  rem();
  delay(2000);
  
  belok_kiri();
  delay(4000);
  
  belok_kanan();
  delay(1500);  
  }
  
  Serial.print("mode = ");
Serial.println(mode);
delay(1000); 
  
  
/*
*/

}
