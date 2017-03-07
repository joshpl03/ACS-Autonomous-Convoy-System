

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


void _mStop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
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
  // Calibration of IR sensors begins immediately
  int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }

  myservo.attach(3);// attach servo on pin 3 to servo object
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  myservo.write(85);//setservo position according to scaled value


  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  _mStop();

  Serial.println("Calibration done!");



}

/*working variables*/

unsigned long lastTime = 0;
double Kp = 2.5, Ki = 0.0, Kd = 3.5;
double Input, Output;
double errSum, lastErr;
double Setpoint = 15;
String inString;

// Line-Following working variables
int lastErr_line = 0;
float KP_line = 0.7, KD_line = 5;

void loop()
{

  middleDistance = Distance_test();

  //Serial.println(Motor_speed);



  //  int num1, num2, num3;
  //  num3 = digitalRead(10); // right sensor (1 means on line no led)
  //  num2 = digitalRead(4); // middle sensor
  //  num1 = digitalRead(2); // left sensor


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

  if ( (temp_value >= 60) && (temp_value <= 180))
    Motor_speed = temp_value;
  if ( (temp_value >= 1000) && (temp_value <= 1070)) {
    temp_value = temp_value - 1000;
    Setpoint = temp_value;
  }
  inString = "";

  // FIXME: Force set vehicle to 'f' state and stable motor speed to 90 (please use this as default speed for start in APP)
//  getstr = 'f';
//  Motor_speed = 90;

  if ( getstr == 'f' || (getstr == 'b') || (getstr == 'l') || (getstr == 'r') || (getstr == 's') || (getstr == 'A') )
    state_of_car = getstr;



  // FIXME: always run this if statement unless vehicle is stopped
  if (state_of_car == 'f' )
  {

    unsigned int sensors[3];
    int position_line = qtr.readLine(sensors);
    //    Serial.println(position_line);
    int error_line = position_line - 3317;
    //      Serial.print("error_line: ");
    //      Serial.print(error_line);
    int set_speed = KP_line * error_line + KD_line * (error_line - lastErr_line);
    //      Serial.print("        set_speed: ");
    //      Serial.print(set_speed);
    lastErr_line = error_line;
    int m1Speed = Motor_speed + set_speed;
    int m2Speed = Motor_speed - set_speed;
    //      Serial.print("        m1Speed: ");
    //      Serial.print(m1Speed);
    //      Serial.print("        m2Speed: ");
    //      Serial.print(m2Speed);
    //      Serial.println("");
//    if (m1Speed < 0) {
//      digitalWrite(in1, LOW);
//      digitalWrite(in2, HIGH);
//      digitalWrite(in3, LOW);
//      digitalWrite(in4, HIGH);
//      m1Speed = -m1Speed;
//    }
//    if (m2Speed < 0) {
//      digitalWrite(in1, HIGH);
//      digitalWrite(in2, LOW);
//      digitalWrite(in3, LOW);
//      digitalWrite(in4, HIGH);
//      m2Speed = -m2Speed;
//    }
//    else {
//      digitalWrite(in1, HIGH);
//      digitalWrite(in2, LOW);
//      digitalWrite(in3, LOW);
//      digitalWrite(in4, HIGH);
//    }
    if (m1Speed > 180)
      m1Speed = 180;
    if (m2Speed > 180)
      m2Speed = 180;
    if (m1Speed < 0)
      m1Speed = 0;
    if (m2Speed < 0)
      m2Speed = 0;

    analogWrite(ENA, m1Speed);
    analogWrite(ENB, m2Speed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

  Serial.println("forward!");




  }
  //  else if (state_of_car == 'b')
  //  {
  //    _mBack();
  //    delay(200);
  //  }
  //  else if (state_of_car == 'l')
  //  {
  //    _mleft();
  //    delay(200);
  //  }
  //  else if (state_of_car == 'r')
  //  {
  //    _mright();
  //    delay(200);
  //  }
  else if (state_of_car == 's')
  {
    _mStop();
  }
  else if (state_of_car == 'A')
  {
    stateChange();
  }



}






