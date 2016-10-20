void setup()
{
    Serial.begin(9600);
}
 
int input;
char ch[3];
void loop()
{
   if( Serial.available())
  {
    Serial.readBytes(ch, 3);
    input = atoi(ch);
    input = input *2;
    Serial.println(input);  
  }
  
    
}
