#include"Arduino.h"
#include "util/delay.h"
#include "avr/io.h" 
#include <EEPROM.h>
//Motor1 -> Right Motor
//Motor2 -> Left Motor
#define DCMOTOR1_ENABLE 2   
#define DCMOTOR2_ENABLE 3
#define DCMOTOR1_INPUT1 7
#define DCMOTOR1_INPUT2 6
#define DCMOTOR2_INPUT1 9
#define DCMOTOR2_INPUT2 8
#define startButton 10

#define most_Left_IR_Digital A8
#define left_IR_Digital A9
#define center_IR_Digital A10
#define right_IR_Digital A11
#define most_Right_IR_Digital A12
#define most_most_Right_IR_Digital 19
#define most_most_Left_IR_Digital 21

#define most_Left_IR_Analog A0
#define left_IR_Analog A1
#define center_IR_Analog A2
#define right_IR_Analog A3
#define most_Right_IR_Analog A4
#define most_most_Right_IR_Analog A14
#define most_most_Left_IR_Analog A15


bool Dvalue1,Dvalue2,Dvalue3,Dvalue4,Dvalue5,Dvalue6,Dvalue7;
int Avalue1,Avalue2,Avalue3,Avalue4,Avalue5,Avalue6,Avalue7;

int l = 0;
int r = 0;
int s = 0;
int u = 0;
int e = 0;
int paths = 0;

bool endFound = 0;

int threshold = 250;
int FT = 20;
int P=0, D=0, I=0, previousError=0, PIDvalue=0, error=0;
int lsp = 100;
int rsp = 100;
int lfspeed = 80;
int turnspeed1 = 255;
int turnspeed2 = 70;
int backspeed1=255;

float Ki = 0;
float Kp = 0.083; 
float Kd = 0.23;

String realPath ="";
int pathCounter =0;


void read_sensors()
{
  Dvalue1 = digitalRead(most_Left_IR_Digital); //1->black  0->white
  Avalue1 = analogRead(most_Left_IR_Analog);
  Dvalue2 = digitalRead(left_IR_Digital);
  Avalue2 = analogRead(left_IR_Analog);
  Dvalue3 = digitalRead(center_IR_Digital);
  Avalue3 = analogRead(center_IR_Analog);
  Dvalue4 = digitalRead(right_IR_Digital);
  Avalue4 = analogRead(right_IR_Analog);
  Dvalue5 = digitalRead(most_Right_IR_Digital);
  Avalue5 = analogRead(most_Right_IR_Analog);
  Dvalue6 = digitalRead(most_most_Left_IR_Digital);
  Avalue6 = analogRead(most_most_Left_IR_Analog);
  Dvalue7 = digitalRead(most_most_Right_IR_Digital);
  Avalue7 = analogRead(most_most_Right_IR_Analog);
}

void print_sensors()
{
  Serial.print("Dvalue1: ");
  Serial.print(Dvalue1);
  Serial.print(", Avalue1: ");
  Serial.print(Avalue1);
  Serial.print(", Dvalue2: ");
  Serial.print(Dvalue2);
  Serial.print(", Avalue2: ");
  Serial.print(Avalue2);
  Serial.print(", Dvalue3: ");
  Serial.print(Dvalue3);
  Serial.print(", Avalue3: ");
  Serial.print(Avalue3);
  Serial.print(", Dvalue4: ");
  Serial.print(Dvalue4);
  Serial.print(", Avalue4: ");
  Serial.print(Avalue4);
  Serial.print(", Dvalue5: ");
  Serial.print(Dvalue5);
  Serial.print(", Avalue5: ");
  Serial.print(Avalue5);
  Serial.print(", Dvalue6: ");
  Serial.print(Dvalue6);
  Serial.print(", Avalue6: ");
  Serial.print(Avalue6);
  Serial.print(", Dvalue7: ");
  Serial.print(Dvalue7);
  Serial.print(", Avalue7: ");
  Serial.println(Avalue7);
}

void turnLeft ()
{
  digitalWrite(DCMOTOR1_INPUT1, HIGH);
  digitalWrite(DCMOTOR2_INPUT1, LOW);
  digitalWrite(DCMOTOR1_INPUT2, LOW);
  digitalWrite(DCMOTOR2_INPUT2, HIGH);
  analogWrite(DCMOTOR1_ENABLE, turnspeed1);
  analogWrite(DCMOTOR2_ENABLE, backspeed1);
  delay(100);
  while (!digitalRead(left_IR_Digital))
  {
    digitalWrite(DCMOTOR1_INPUT1, HIGH);
    digitalWrite(DCMOTOR2_INPUT1, LOW);
    digitalWrite(DCMOTOR1_INPUT2, LOW);
    digitalWrite(DCMOTOR2_INPUT2, HIGH);
    analogWrite(DCMOTOR1_ENABLE, turnspeed2);
    analogWrite(DCMOTOR2_ENABLE, turnspeed2);
  }
  analogWrite(DCMOTOR1_ENABLE, 0);
  analogWrite(DCMOTOR2_ENABLE, 0);
  _delay_ms(50);
}

void turnRight ()
{
  digitalWrite(DCMOTOR1_INPUT1, LOW);
  digitalWrite(DCMOTOR2_INPUT1, HIGH);
  digitalWrite(DCMOTOR1_INPUT2, HIGH);
  digitalWrite(DCMOTOR2_INPUT2, LOW);
  analogWrite(DCMOTOR1_ENABLE, backspeed1);
  analogWrite(DCMOTOR2_ENABLE, turnspeed1);
  delay(100);
  while (!digitalRead(right_IR_Digital))
  {
    digitalWrite(DCMOTOR1_INPUT1, LOW);
    digitalWrite(DCMOTOR2_INPUT1, HIGH);
    digitalWrite(DCMOTOR1_INPUT2, HIGH);
    digitalWrite(DCMOTOR2_INPUT2, LOW);
    analogWrite(DCMOTOR1_ENABLE, turnspeed2);
    analogWrite(DCMOTOR2_ENABLE, turnspeed2);
  }
  analogWrite(DCMOTOR1_ENABLE, 0);
  analogWrite(DCMOTOR2_ENABLE, 0);
  _delay_ms(50);
}

void littleForward ()
{
  digitalWrite(DCMOTOR1_INPUT1, HIGH);
  digitalWrite(DCMOTOR2_INPUT1, HIGH);
  analogWrite(DCMOTOR1_ENABLE, 70);
  analogWrite(DCMOTOR2_ENABLE, 70);
  _delay_ms(50);
  stop();
}
void stop ()
{
  digitalWrite(DCMOTOR1_INPUT1, HIGH);
  digitalWrite(DCMOTOR2_INPUT1, HIGH);
  digitalWrite(DCMOTOR1_INPUT2, LOW);
  digitalWrite(DCMOTOR2_INPUT2, LOW);
  analogWrite(DCMOTOR1_ENABLE, 0);
  analogWrite(DCMOTOR2_ENABLE, 0);
}
void uturn ()
{
  while (!digitalRead(center_IR_Digital))
  {
    digitalWrite(DCMOTOR1_INPUT1, LOW);
    digitalWrite(DCMOTOR2_INPUT1, HIGH);
    digitalWrite(DCMOTOR1_INPUT2, HIGH);
    digitalWrite(DCMOTOR2_INPUT2, LOW);
    analogWrite(DCMOTOR1_ENABLE, turnspeed2);
    analogWrite(DCMOTOR2_ENABLE, turnspeed2);
  }
  analogWrite(DCMOTOR1_ENABLE, 0);
  analogWrite(DCMOTOR2_ENABLE, 0);
  delay(50);
}

void linefollow()
{ 
  paths = 0;
  while (!digitalRead(most_most_Left_IR_Digital) && !digitalRead(most_most_Right_IR_Digital) && (digitalRead(center_IR_Digital) || digitalRead(left_IR_Digital) || digitalRead(right_IR_Digital)||digitalRead(most_Left_IR_Digital) || digitalRead(most_Right_IR_Digital)))

  {
    lfspeed = 80; //line follow speed
    PID();
  }
}

void PID()
{
  int error = (analogRead(left_IR_Analog)+analogRead(most_Left_IR_Analog)) - (analogRead(right_IR_Analog)+ analogRead(most_Right_IR_Analog));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  rsp = lfspeed + PIDvalue;
  lsp = lfspeed - PIDvalue;

  if (lsp > 250) {
    lsp = 250;
  }
  if (lsp < 70) {
    lsp = 70;
  }
  if (rsp > 250) {
    rsp = 250;
  }
  if (rsp < 80) {
    rsp = 80;
  }

  digitalWrite(DCMOTOR1_INPUT1, HIGH);
  digitalWrite(DCMOTOR2_INPUT1, HIGH);
  digitalWrite(DCMOTOR1_INPUT2, LOW);
  digitalWrite(DCMOTOR2_INPUT2, LOW);
  analogWrite(DCMOTOR1_ENABLE, rsp);
  analogWrite(DCMOTOR2_ENABLE, lsp);
}

void checknode ()
{
  l = 0;
  r = 0;
  s = 0;
  u = 0;
  e = 0;
  paths = 0;

  if (digitalRead(most_most_Right_IR_Digital)){ 
  r = 1; 
  }
  if (digitalRead(most_most_Left_IR_Digital)){
  l = 1;
  }
  if (((digitalRead(most_Left_IR_Digital))) && ((digitalRead(left_IR_Digital)))&& ((digitalRead(right_IR_Digital))) && ((digitalRead(most_Right_IR_Digital))) && ((digitalRead(center_IR_Digital))) &&digitalRead(most_most_Left_IR_Digital)&& digitalRead(most_most_Right_IR_Digital)) {
  e = 1;
  }
   if (!digitalRead(most_Left_IR_Digital) && !digitalRead(left_IR_Digital)&& !digitalRead(right_IR_Digital) && !digitalRead(most_Right_IR_Digital) && !digitalRead(center_IR_Digital)&&!digitalRead(most_most_Left_IR_Digital)&& !digitalRead(most_most_Right_IR_Digital)) {
    u = 1;
  }

  if (u == 0)
  {
    for (int i = 0; i < FT; i++)
    {
      lfspeed = 70;
      PID();
      if (digitalRead (most_most_Right_IR_Digital)) r = 1;
      if (digitalRead (most_most_Left_IR_Digital)) l = 1;
    }

    littleForward();
    stop();
    _delay_ms(150);
    if (u == 0)
    {
      if (digitalRead(center_IR_Digital) || digitalRead(left_IR_Digital) || digitalRead(right_IR_Digital)) s=1;
    }
  }
  paths = l + s + r;
}

void navigate()
{
  if (e == 1)
  {
    _delay_ms(200);
    if (((digitalRead(most_Left_IR_Digital))) && ((digitalRead(left_IR_Digital)))&& ((digitalRead(right_IR_Digital))) && ((digitalRead(most_Right_IR_Digital))) && ((digitalRead(center_IR_Digital))) &&digitalRead(most_most_Left_IR_Digital)&& digitalRead(most_most_Right_IR_Digital)){
    realPath += 'E';
    endFound = 1;
    stop();
    _delay_ms(10000);
    }
  }
  else if (r == 1)
  {
    stop();
    _delay_ms(200);
    realPath += 'R';
    turnRight();
  }
  else if (l == 1&&s==0)
  {
    stop();
    _delay_ms(200);
    realPath += 'L';
    turnLeft();
  }
  else if (u == 1)
  {
    realPath += 'U';
    stop();
    _delay_ms(200);
    uturn();
  }
  else
  {
    realPath += 'S';
  }
}

void navigate2(char node)
{
 switch(node) {
    case 'E':
        stop();
        _delay_ms(10000);
        break;
    case 'R':
        stop();
        _delay_ms(200);
        turnRight();
        break;
    case 'L':
        stop();
        _delay_ms(200);
        turnLeft();
        break;
    case 'U':
        stop();
        _delay_ms(200);
        uturn();
        break;
    default:
        break;
}

void storeRealPathToEEPROM() {
  int len = realPath.length();
  EEPROM.write(0, len);  // Store the length at the first byte
  for (int i = 0; i < len; i++) {
    EEPROM.write(i + 1, realPath[i]);
  }
}

void retrieveRealPathFromEEPROM() {
  int len = EEPROM.read(0);  // Read the length from the first byte
  char path[len + 1];
  for (int i = 0; i < len; i++) {
    path[i] = EEPROM.read(i + 1);
  }
  path[len] = '\0';
  realPath = String(path);
}


void setup() {
  Serial.begin(9600);
  pinMode(startButton, INPUT_PULLUP);
  pinMode (DCMOTOR1_INPUT1, OUTPUT);
  pinMode (DCMOTOR2_INPUT1, OUTPUT);
  pinMode (DCMOTOR1_INPUT2, OUTPUT);
  pinMode (DCMOTOR2_INPUT2, OUTPUT);
  _delay_ms(2000);
}

void loop() {
  while (digitalRead(startButton) == 1) //pull up
  { 
  }
  _delay_ms(200);
  while (endFound == 0)
  {
    linefollow(); 
    checknode();
    stop();
    _delay_ms(50);
    navigate ();
  }
  storeRealPathToEEPROM();
  

  while (digitalRead(startButton) == 1)
  { 
  }
  retrieveRealPathFromEEPROM();
  for (int x = 0; x < 5; x++)
  {
    realPath.replace("RURUS", "U");
    realPath.replace("RUSUR", "U");
    realPath.replace("RUR", "S");
    realPath.replace("SUR", "L");
    realPath.replace("RUS", "L");
    realPath.replace("RUL", "U");
    realPath.replace("LUR", "U");
  }

  while(1){  
        linefollow(); 
        littleForward();
        stop();
        _delay_ms(150);
        if (pathCounter < realPath.length()) {
            navigate2(realPath[pathCounter]);
            pathCounter++;
       } else {
      stop();
      break;  // End of path
    }
  }

}