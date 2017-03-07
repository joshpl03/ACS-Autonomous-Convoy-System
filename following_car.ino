


#include <Servo.h> //servo library
Servo myservo; // create servo object to control servo
int Echo = A4;  
int Trig = A5; 
int rightDistance = 0,leftDistance = 0,middleDistance = 0 ;


int LED=13;
volatile int state = LOW;
char getstr;
int in1=6;
int in2=7;
int in3=8;
int in4=9;
int ENA=5;
int ENB=11;
int ABS=150;


int Base_speed=75;
int Motor_speed = 0;
int PID_Speed_increase = 0;

char state_of_car = 'm';

void _mForward()

{ 
  analogWrite(ENA,Motor_speed);
  analogWrite(ENB,Motor_speed);
  digitalWrite(in1,HIGH);//digital output
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  //Serial.println("Forward");
}
void _mBack()
{
  analogWrite(ENA,Base_speed);
  analogWrite(ENB,Base_speed);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("Back");
}

void _mleft()
{
  analogWrite(ENA,Motor_speed);
  analogWrite(ENB,Motor_speed);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW); 
  //Serial.println("go left!");
}

void _m_curve_left()
{
  analogWrite(ENA,Motor_speed+15);
  analogWrite(ENB,Motor_speed-15);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH); 
  //Serial.println("go slight left!");
}

void _mright()
{
  analogWrite(ENA,Motor_speed-15);
  analogWrite(ENB,Motor_speed+15);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  //Serial.println("go right!");
}
void _m_curve_right()
{
  analogWrite(ENA,Motor_speed-15);
  analogWrite(ENB,Motor_speed+15);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH); 
  //Serial.println("go slight right!");
}
void _mStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  //Serial.println("Stop!");
}
void stateChange()
{
  state = !state;
  digitalWrite(LED, state);  
}

 /*Ultrasonic distance measurement Sub function*/
int Distance_test()   
{
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance/58;

  return (int)Fdistance;
}  


void setup()
{ 
  myservo.attach(3);// attach servo on pin 3 to servo object
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  myservo.write(85);//setservo position according to scaled value

  
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  _mStop();

  
}

/*working variables*/

unsigned long lastTime = 0;
double Kp=2.5, Ki=0.0, Kd=3.5;
double Input, Output;
double errSum, lastErr;
double Setpoint = 15;
String inString;

void loop()
{ 
  
  /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
   
   
   
/*Compute all the working error variables*/
   middleDistance = Distance_test();
   Input = middleDistance;
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = Kp * error + Ki * errSum + Kd * dErr;

   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
   
   
   
   




  Motor_speed = Base_speed - Output;
  Motor_speed = constrain(Motor_speed, 60, 130);


  Serial.println(Motor_speed);

  
  
  int num1,num2,num3; 
  num3=digitalRead(10); // right sensor (1 means on line no led)
  num2=digitalRead(4); // middle sensor
  num1=digitalRead(2); // left sensor


while (Serial.available() > 0) {
  getstr=Serial.read();
  char lmao = getstr;
    if (isDigit(lmao)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)lmao;
    }  
  }
  int temp_value = inString.toInt();
  
  if( (temp_value >= 60) && (temp_value <= 180))
  Motor_speed = temp_value;
  if( (temp_value >= 1000) && (temp_value <= 1070)){
  temp_value = temp_value - 1000;
  Setpoint = temp_value;
  }
  inString = "";


  Serial.println(Setpoint);


  if( getstr == 'f' || (getstr == 'b') || (getstr == 'l') || (getstr == 'r') || (getstr == 's') || (getstr == 'A') )
  state_of_car = getstr;


   if(state_of_car=='f')
   {

         if(middleDistance<=15)
          {     
            _mStop();
          }
         
         else if(middleDistance > 15)
         {
                  if((num1==0)&&num2&&num3) 
                    {
                     _m_curve_left();          //The sensor detected that right car turn left immediately when it meets black line 
                         delay(2);
                         while(1){
                            num2=digitalRead(2);         //Cycle to judge degree of intermediate sensor,
                            
                            if(num2==1)
                              {  _m_curve_left();                     //If num2==1 does not go to the middle position, continue to turn left.
                                 delay(2);}
                                 else
                                 break;                      //Detection of num2==0 instructions turned over, out of the loop, detection of three sensorsâ€™ statusand then make appropriate action
                                 }                                       //The following and so on
                    }    
            
                  else if((num1==0)&&(num2 == 0)&&num3) 
                    {
                       _mleft();          //The sensor detected that right car turn left immediately when it meets black line 
                         delay(2);
                         while(1){
                          num2=digitalRead(2);         //Cycle to judge degree of intermediate sensor,
                          
                          if(num2==1)
                            {   _mleft();                     //If num2==1 does not go to the middle position, continue to turn left.
                               delay(2);}
                               else
                               break;                      //Detection of num2==0 instructions turned over, out of the loop, detection of three sensorsâ€™ statusand then make appropriate action
                               }                                       //The following and so on
                    }    
            
                    
                    else if(num1&&num2&&(num3==0))
                      {
                        _mright();
                        delay(2);
                        while(1)
                                {
                                 num2=digitalRead(2);
                                 
                                 if(num2==1){
                                    _mright();  
                                    delay(2);}
                                 
                                 else
                                  break;
                                }
                      }
            
                    else if(num1&&(num2==0)&&(num3==0))
                      {
                        _mright();
                        delay(2);
                        while(1)
                                {
                                 num2=digitalRead(2);
                                 
                                 if(num2==1){
                                    _mright();  
                                    delay(2);}
                                 
                                 else
                                  break;
                                }
                      }
                      else if((num1==0)&& (num2) && (num3==0))
                      {
                        _mForward();
                        delay(2);
                      }
                              
                     else
                       {
                         _mForward(); 
                         delay(2);
                       } 
         }
         
         
  }
  else if(state_of_car=='b')
  {
    _mBack();
    delay(200);
  }
  else if(state_of_car=='l')
  {
    _mleft();
    delay(200);
  }
  else if(state_of_car=='r')
  {
    _mright();
    delay(200);
  }
  else if(state_of_car=='s')
  {
     _mStop();     
  }
  else if(state_of_car=='A')
  {
  stateChange();
  }


  
}


