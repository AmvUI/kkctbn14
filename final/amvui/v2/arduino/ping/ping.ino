#include <Servo.h>

int echoPin_dka = 24; // Echo Pin        dka = depan kanan
int trigPin_dka = 22; // Trigger Pin

int echoPin_dki = 36; // Echo Pin
int trigPin_dki = 34; // Trigger Pin


long duration_dka, duration_dki;
long distance_dka, distance_dki; // Duration used to calculate distance

Servo s1, s2, s3, s4;
Servo escl, escr;
int x, y, ki_belok, ka_belok, s_ki, s_ka;

void setup() {
 Serial.begin (115200);
 pinMode(trigPin_dka, OUTPUT);
 pinMode(echoPin_dka, INPUT);

 pinMode(trigPin_dki, OUTPUT);
 pinMode(echoPin_dki, INPUT);

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

void loop() {

//int duration_dka, duration_dki, duration_bka, duration_bki; // Duration used to calculate distance
//int distance_dka, distance_dki, distance_bka, distance_bki; // Duration used to calculate distance

//ini gw ga ngerti apa maksudnya. tp dg ada ini, proses di serial monitor jd lbh normal  
 digitalWrite(trigPin_dka, LOW);  digitalWrite(trigPin_dki, LOW);
 delayMicroseconds(10); 

 digitalWrite(trigPin_dka, HIGH);  digitalWrite(trigPin_dki, HIGH);
 delayMicroseconds(60);  
 
 digitalWrite(trigPin_dka, LOW);  digitalWrite(trigPin_dki, LOW);

//baca nilai sensor dalam bentuk durasi 
 duration_dka = pulseIn(echoPin_dka, HIGH); duration_dki = pulseIn(echoPin_dki, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
 distance_dka = duration_dka/58.2; distance_dki = duration_dki/58.2;

 // perhitungan selisih nilai dari sr04 x untuk depan, y untuk belakang
 x = distance_dka - distance_dki;
 
 // bikin batas sudut belok dari x dan y
 if (x>=100){
   x=100;
 } else if (x<=-100) {
 x=-100;
 }
  
 if (y>=100){
   y=100;
 } else if (y<=-100){
 y=-100;
 }
 
 // perhitungan besar sudut servo belok kiri dan kanan;
 //'60' posisi lurus nozzle kiri; '1/5' dan '1/10' koef kira2 dr perubahan sr04 depan dan sr04 belakang
 //'110' posisi lurus nozzle kanan
 ki_belok = 60 + x/15; 
 ka_belok = 110 + x/10; 
 
 //perhitungan speed motor kiri dan kanan
 // '5' kira2 gw aja, 1 cm perubahan nilai x, nambah speed motor 5
/* if(x>=10) {
 s_ki = 900 - x*5;
 s_ka = 900;
 } else if (x<10) {
 s_ki = 900;
 s_ka = 900 - x*5;
 } else {
 s_ki = s_ka = 900;
 } */
 
 // '5' dan '10' sudut nozzle mundur, selalu posisinya maju
 // 1000 speed motor, harusnya ikut berubah, tp belom kebayang rumusnya
 jalan(0, 0, 85, ki_belok, 100, ka_belok);
 
 // nampilin sudut belok di serial monitor
 Serial.print("dka = ");  Serial.print(distance_dka); Serial.print(" | ");
 Serial.print("y = ");  Serial.print(y); Serial.println(" ");
 
 Serial.print("ki_b = "); Serial.print(ki_belok); Serial.print(" | ");
 Serial.print("ka_b = "); Serial.print(ka_belok); Serial.println(" ");
 
 delay(200);

//Serial.println(distance_dka);

}
