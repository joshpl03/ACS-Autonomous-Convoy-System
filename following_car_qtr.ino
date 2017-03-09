#include <QTRSensors.h>
#include <Servo.h> //servo library
QTRSensorsAnalog qtr((unsigned char[]) {
  8, 9, 10, 11, 12, 13, 14, 15
}, 8);
Servo myservo; // create servo object to control servo
int Echo = A4;
int Trig = A5;
int rightDistance = 0, leftDistance = 0, middleDistance = 0 ;


int LED = 13;
volatile int state = LOW;
char getstr;
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 5;
int ENB = 11;
int ABS = 150;


int Base_speed = 80;
int Motor_speed = 0;
int PID_Speed_increase = 0;

char state_of_car = 'm';
int temp_motor_speed = 80;

void _mStop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
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
  Fdistance = Fdistance / 58;

  return (int)Fdistance;
}


void setup()
{
  myservo.attach(3);// attach servo on pin 3 to servo object
  // Calibration of IR sensors begins immediately
  int i;
  for (i = 0; i < 100; i++)  // make the calibration take about 2 seconds
  {
    qtr.calibrate();
    delay(20);
  }

  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);


  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  _mStop();

  //Serial.println("Calibration done!");
  digitalWrite(LED, HIGH);
  Motor_speed = Base_speed;

  myservo.write(90);
}

/*working variables*/

unsigned long lastTime = 0;
double Kp = 2.5, Ki = 0.0, Kd = 3.5;
//double Kp = 0.6, Ki = 0.0, Kd = 3;
double Input, Output;
double errSum, lastErr;
double Setpoint = 15;
String inString;
bool PD_straight = true;
long deb_delay = 2000;
long last_deb = 0;


// Line-Following working variables
int lastErr_line = 0;
float KP_line = 0.05, KD_line = 0.07;
int lastErr_angle = 0;
int last_angle = 90;
float KP_angle = 0.1, KD_angle = 0;
int m1Speed = 0;
int m2Speed = 0;

void loop()
{


  //Serial.println(Motor_speed);


  while (Serial.available() > 0) {
    getstr = Serial.read();
    char lmao = getstr;
    if (isDigit(lmao)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)lmao;
    }
  }
  int temp_value = inString.toInt();


  if ( (temp_value >= 60) && (temp_value <= 180)){
    
    Motor_speed = temp_value;
    temp_motor_speed = Motor_speed;
  }
  
  if ( (temp_value >= 1000) && (temp_value <= 1070)) {
    temp_value = temp_value - 1000;
    Setpoint = temp_value;
  }
  inString = "";


  //Serial.println(Setpoint);


  if ( getstr == 'f' || (getstr == 'b') || (getstr == 'l') || (getstr == 'r') || (getstr == 's') || (getstr == 'A') )
    state_of_car = getstr;

  state_of_car = 'f';

  if (state_of_car == 'f')
  {
    
    
        /*Compute all the working error variables*/
    middleDistance = Distance_test();
    Input = middleDistance;
    double error = Input - Setpoint;

    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);


    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;



    /*Compute PID Output*/
    Output = Kp * error + Ki * errSum + Kd * dErr;
    lastErr = error;
    lastTime = now;

Motor_speed = Base_speed + Output;
    
    
    
    
    // float KP_line = 0.05, KD_line = 1;
    unsigned int sensors[8];
    int position_line = qtr.readLine(sensors);
    //        Serial.print("position_line: ");
    //        Serial.println(position_line);
    unsigned int sensor_values[8];
    qtr.readCalibrated(sensor_values);

    int j, total = 0;
    for (j = 0; j < 8; j++)  // make the calibration take about 5 seconds
    {
      //      Serial.print("Sen");
      //      Serial.print(j);
      //      Serial.print(": ");
      //      Serial.print(sensors[j]);
      //      Serial.print("  ");
      total += sensors[j];
    }
    //    Serial.println("");
    int avg = total / 8;
    
//Serial.print(millis() - last_deb);
//Serial.print("  ");
    if ( (millis() - last_deb) > deb_delay )
    {
      if (avg > 900 && PD_straight)
      {
        Motor_speed = 65;
        PD_straight = !PD_straight;
        last_deb = millis();
      }
      else if (avg > 900 && !PD_straight)
      {
        Motor_speed = temp_motor_speed;

        PD_straight = !PD_straight;
        last_deb = millis();
      }
    }


    if(!PD_straight)
    Motor_speed = 65;
    
    
    //    Serial.print("position_line: ");
    //    Serial.print(position_line);
    int error_line = position_line - 3116;
    //        Serial.print("        error_line: ");
    //        Serial.print(error_line);
    int set_speed = KP_line * error_line + KD_line * (error_line - lastErr_line);
    //    Serial.print("        set_speed: ");
    //    Serial.print(set_speed);
    lastErr_line = error_line;
    m1Speed = Motor_speed + set_speed;
    m2Speed = Motor_speed - set_speed;



    m1Speed = constrain(m1Speed, 0, 180);
    m2Speed = constrain(m2Speed, 0, 180);




    /*How long since we last calculated*/



    

    analogWrite(ENA, m1Speed);
    analogWrite(ENB, m2Speed);

    if (middleDistance <= Setpoint)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      //    Serial.println("Stop");
    }
    else
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }

    //Serial.println("forward!");

    //    //Serial.println(angle);
    //    int angle = int(double(position_line) / 7000 * 180);
    //    int error_angle = angle - last_angle;
    //    int set_angle = KP_angle * error_angle + KD_angle * (error_angle - lastErr_angle);
    //    lastErr_angle = error_angle;
    //    angle = last_angle + set_angle;
    //    last_angle = angle;
    //    myservo.write(angle);
    //    delay(50);


  }
  else if (state_of_car == 's')
  {
    _mStop();
  }
  else if (state_of_car == 'A')
  {
    stateChange();
  }

}




