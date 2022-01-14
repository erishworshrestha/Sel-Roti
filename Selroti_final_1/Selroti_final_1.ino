#include <Servo.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <EEPROM.h>

int refill_time = 3; //In seconds
const int rs = 39, en = 41, d4 = 49, d5 = 47, d6 = 45, d7 = 43;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const byte ROWS = 4;
const byte COLS = 4;
char customKey ;

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {37, 35, 33, 31};
byte colPins[COLS] = {23, 25, 27, 29};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

int inputCol = 0, inputRow = 1  ;
String inputData;

SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);

int limitSwitchStart = A0;
int limitSwitchMiddle = A2;
int limitSwitchEnd = A4;
int limitSwitchPanEnd = A7;
int limitSwitchPan = A8;
int limitSwitchMain = A11;
int selCount = 0;
int cleaningButton = 53;
int refill_relay = 26;

#define STEPPIN 4
#define DIRPIN 5
#define ENAPIN 6
#define servo_ 7

#define address 0x80

unsigned long temp, counter = 0;
int initial_stepper_speed_ = 1500;
int stepper_speed_ = 800;
int servo_speed = 1;
int pos = 0;

int buzzer = 14;
int revolution_ = 4150;
int before = 900;
int currentSpeed =  stepper_speed_;

int timer ;
int flipTime = 5000;
int countTime = 0;
int motorSpeed = 125;
bool endTask = true;
bool cleaningMode = false;
bool pause_ = false;
int cleaningTimer = 2000;
int pressLength;
int alarmLength = 0;
int resumeTimer = 1000;
bool runningMode = false;
Servo nozzleServo, flipFirstServo, flipSecondServo;

const int STEPTIME = 1;

int thickness = 90;
int upperThicknessLimit = 115;
int lowerThicknessLimit = 75;
bool cleaningModeBool = false;

void setup()
{
  Serial.begin(9600);
  roboclaw.begin(38400);

  nozzleServo.attach(servo_);
  flipFirstServo.attach(16);
  flipSecondServo.attach(15);
  pinMode(servo_, INPUT);
  pinMode(16, INPUT);
  pinMode(15, INPUT);

  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(ENAPIN, OUTPUT);

  pinMode(limitSwitchStart , INPUT);
  pinMode(limitSwitchMiddle  , INPUT);
  pinMode(limitSwitchEnd  , INPUT_PULLUP);
  pinMode(limitSwitchPanEnd  , INPUT);
  pinMode(limitSwitchPan, INPUT);
  pinMode(limitSwitchMain, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(refill_relay, OUTPUT);
  digitalWrite(refill_relay, LOW);

  pinMode(cleaningButton, INPUT_PULLUP);

  lcd.begin(20, 4);
  lcd.setCursor(2, 1);
  lcd.print("Calibrating...");
  necessaryChecks();
  endTask = EEPROM.read(2);

  if (endTask == false)
  {
    printPleaseWait();
    endTaskModeMain();
  }

  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Welcome");
  lcd.setCursor(8, 1);
  lcd.print("to");
  lcd.setCursor(2, 2);
  lcd.print("Sel-Roti Machine");
  delay(2000);
  rotiThickness();
  delay(2000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Enter the amount : ");
  hint();
}

//*****************************************************Beginning of Loop*****************************************************
void loop()
{
    pinMode(servo_, INPUT);
  pinMode(16, INPUT);
  pinMode(15, INPUT);
  runningMode = false;
  cleaningModeMain();
  customKey = customKeypad.getKey();
  input();

  if (analogRead(limitSwitchStart) > 50)
  {
    digitalWrite(refill_relay, HIGH);
  }
  else
  {
    digitalWrite(refill_relay, LOW);
  }

  if (endTask == false && inputData.toInt() != 0 )
  {
    runningMode = true;
    displayAmountData();
    necessaryChecks();
    //    refill();

    while (analogRead(limitSwitchMiddle) > 50)
    {
      mainMotorMovement();
    }

    mainMotorStop();
    pinMode(servo_, OUTPUT);
    forward(revolution_);
    pinMode(servo_, INPUT);

    if (endTask == false)
      flipSecond();

    if (endTask == false)
    {
      while (analogRead(limitSwitchEnd) > 50)
      {
        mainMotorMovement();
      }
      mainMotorStop();
      pinMode(servo_, OUTPUT);
      forward(revolution_);
      pinMode(servo_, INPUT);

      if (endTask == false)
        flipThird();

      mainMotorStop();
      calibrate_();
      mainMotorStop();


      if (endTask == false)
      {
        alarm_();
        flipFirst();
        //        refill();
      }
    }
  }
    pinMode(servo_, INPUT);
  pinMode(16, INPUT);
  pinMode(15, INPUT);
}

//*****************************************************End of Loop*****************************************************

void stepperReset()
{
  digitalWrite(ENAPIN, LOW);
  digitalWrite(DIRPIN, HIGH);
  //  digitalWrite(DIRPIN, HIGH);
  while (analogRead(limitSwitchMain) > 50)
  {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(STEPTIME);
    digitalWrite(STEPPIN, LOW);
    delayMicroseconds(STEPTIME);
    delayMicroseconds( stepper_speed_ );
  }
}

void actuatorOn() {
  for (pos = 0 ; pos <= thickness ; pos += 1)
  {
    nozzleServo.write(pos);
  }
}

void actuatorOff() {
  for (pos = thickness ; pos >= 0 ; pos -= 1)
  {
    nozzleServo.write(pos);
  }
}

void forward(int steps)
{
  int i;
  digitalWrite(ENAPIN, LOW);
  digitalWrite(DIRPIN, HIGH);

  for (i = 0; i < steps; i++) {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(STEPTIME);
    digitalWrite(STEPPIN, LOW);
    delayMicroseconds(STEPTIME);

    if (i == 100)
      actuatorOn();

    if (i < 450)
      delayMicroseconds(initial_stepper_speed_ - i * 2.222);
    else
      delayMicroseconds(stepper_speed_);

    if (i == 3800)
      actuatorOff();
  }

  if (analogRead(limitSwitchEnd) > 50)
    roboclaw.BackwardM2(address, 100);

  if (analogRead(limitSwitchEnd) < 50)
    roboclaw.ForwardM2(address, motorSpeed);

  for (i = 0; i < before; i++) {
    digitalWrite(STEPPIN, HIGH);
    delayMicroseconds(STEPTIME);
    digitalWrite(STEPPIN, LOW);
    delayMicroseconds(STEPTIME);
    delayMicroseconds( currentSpeed );
    currentSpeed += 2;
  }
  currentSpeed = stepper_speed_;

  EEPROM.write(2, endTask);
  stepperReset();
  selCount++;
  displayAmountData();

  if (selCount == inputData.toInt())
    endTaskModeMain();
}

void calibrate_()
{
    pinMode(servo_, INPUT);
  while ( analogRead(limitSwitchStart) > 50)
  {
    roboclaw.ForwardM2(address, motorSpeed);
  }
  mainMotorStop();
}

void alarm_() {
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
}

void okAlarm_()
{
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(100);
}


void panCheck()
{

  while (analogRead(limitSwitchPan) > 50)
  {
    roboclaw.BackwardM1(address, 23);
    cleaningModeMain();
  }
  roboclaw.BackwardM1(address, 0);
}

void flipFirst()
{
  displayAmountData();
  while (analogRead(limitSwitchPanEnd) > 50)
  {
    roboclaw.ForwardM1(address, 30);
  }
  panCheck();
}

void flipSecond()
{
  pinMode(16, OUTPUT);
  for (int i = 0; i <= 170; i += 1)
  {
    flipFirstServo.write(i);
    delay(10);
  }
  delay(500);
  for (int i = 170; i >= 0; i -= 1)
  {
    flipFirstServo.write(i);
    delay(10);
  }
  pinMode(16, INPUT);
}

void flipThird()
{
  pinMode(15, OUTPUT);
  for (int i = 0; i <= 170; i += 1)
  {
    flipSecondServo.write(i);
    delay(10);
  }
  delay(500);
  for (int i = 170; i >= 0; i -= 1)
  {
    flipSecondServo.write(i);
    delay(10);
  }
  pinMode(15, INPUT);
}

void necessaryChecks()
{
  stepperReset();
  nozzleServo.write(10);

  for (int i = 20; i >= 0; i -= 1)
  {
    flipSecondServo.write(i);
    flipFirstServo.write(i);
    delay(10);
  }

  pinMode(servo_, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);

  calibrate_();
  panCheck();
  mainMotorStop();
  okAlarm_();
}

void mainMotorMovement()
{
  roboclaw.BackwardM2(address, motorSpeed);
}

void mainMotorStop()
{
  roboclaw.BackwardM2(address, 0);
}

void cleaningModeMain()
{
  if ( runningMode == false )
  {
    bool stopPan = true;
    pressLength = 0;
    alarmLength = 0;
    while (digitalRead(cleaningButton) == 0)
    {
      delay(100);
      pressLength += 100;
      if (pressLength - alarmLength == 1000)
      {
        alarm_();
        alarmLength = pressLength;
      }
    }

    if (pressLength >= resumeTimer && pressLength < cleaningTimer)
    {
      if (cleaningModeBool == true)
      {
        okAlarm_();
        lcd.clear();
        lcd.setCursor(4, 1);
        lcd.print("Resuming....");
        if (endTask == true)
        {
          lcd.setCursor(4, 4);
          lcd.print("C : Continue");
        }
        cleaningModeBool = false;
        panCheck();
        pinMode(15, OUTPUT);
        pinMode(16, OUTPUT);
        for (int i = 90; i >= 0; i -= 1)
        {
          flipSecondServo.write(i);
          flipFirstServo.write(i);
          delay(10);
        }
        necessaryChecks();
        pinMode(15, INPUT);
        pinMode(16, INPUT);
      }
    }
    else if (pressLength >= cleaningTimer)
    {
      okAlarm_();
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Cleaning Mode");
      cleaningModeBool = true;
      pinMode(15, OUTPUT);
      pinMode(16, OUTPUT);
      for (int i = 0; i <= 100; i += 1)
      {
        flipSecondServo.write(i);
        flipFirstServo.write(i);
      }
      for (int i = 0; i <= 100; i += 1)
      {
        roboclaw.ForwardM1(address, 26);
        delay(10);
      }
      pinMode(15, INPUT);
      pinMode(16, INPUT);
      while (stopPan == true)
      {
        roboclaw.ForwardM1(address, 0);
        if (digitalRead(cleaningButton) == 0)
        {
          stopPan = false;
        }
      }
    }
    else
    {
      alarmLength = 0;
    }
  }
}

void endTaskModeMain()
{
  calibrate_();
  flipThird();
  alarm_();
  printPleaseWait();
  necessaryChecks();
  if (endTask == false)
  {
    endTask = true;
    alarm_();
    //     delay(500);
    //    flipSecond();
    delay(1000);
    flipFirst();
    delay(8000);
    flipSecond();
    delay(5000);
    flipThird();
    okAlarm_();
  }
  lcd.clear();
  lcd.setCursor(7, 1);
  lcd.print("COOKED");
  lcd.setCursor(0, 3);
  lcd.print("C : Start Again");
  mainMotorStop();
  EEPROM.write(2, endTask);
  selCount = 0;
}

void input()
{
  cleaningModeMain();
  if (customKey)
  {
    if (customKey != 'B' || customKey != '*' || customKey != '#')
    {
      switch (customKey)
      {
        case 'A':
          lcd.clear();
          if (inputData != "" && inputData.toInt() != 0)
          {

            lcd.setCursor(3, 1);
            necessaryChecks();
            displayAmountData();
            endTask = false;
          }
          else
          {
            lcd.setCursor(1, 0);
            lcd.print("Enter the amount : ");
            hint();
          }

          break;

        case 'C':
          lcd.clear();
          inputData = "";
          lcd.setCursor(1, 0);
          lcd.print("Enter the amount : ");
          lcd.setCursor(0, 1) ;
          lcd.print("                          ");
          hint();
          break;

        case 'D':
          lcd.clear();
          inputData = "";
          lcd.setCursor(1, 0);
          lcd.print("Enter the amount : ");
          lcd.setCursor(0, 1) ;
          lcd.print("                          ");
          selCount = 0;
          hint();
          break;

        default:
          lcd.clear();
          hint();
          lcd.setCursor(1, 0);
          lcd.print("Enter the amount : ");
          if (customKey != '*' && customKey != '#' && customKey != 'A' && customKey != 'B' && customKey != 'C' && customKey != 'D')
          {
            inputData = inputData + customKey;
          }
          lcd.setCursor(0, 1) ;
          lcd.print(inputData);
          inputCol++;
          break;
      }
    }
  }
}

void hint()
{
  lcd.setCursor(0, 3);
  lcd.print("A: START | C: Clear");
}

void displayAmountData()
{
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Cooking.....");
  lcd.setCursor(0, 1);
  lcd.print("Sel Count :");
  lcd.setCursor(12, 1);
  lcd.print(selCount);
  lcd.setCursor(0, 2);
  lcd.print("Remaining :");
  lcd.setCursor(12, 2);
  lcd.print(inputData.toInt() - selCount);
}

void rotiThickness()
{
  //  if (endTask == false)
  //  {
  lcd.clear();
  customKey = customKeypad.getKey();
  unsigned long startTimer = millis();
  while ( customKey != 'A')
  {
    customKey = customKeypad.getKey();
    lcd.setCursor(0, 0);
    lcd.print("Thickness : ");
    lcd.setCursor(12, 0);
    lcd.print(thickness);
    lcd.setCursor(0, 1);
    lcd.print("A :Long Press(Start)");
    lcd.setCursor(0, 2);
    lcd.print("B : Increase");
    lcd.setCursor(0, 3);
    lcd.print("C : Decrease");


    if (customKey == 'B')
    {
      if (thickness < upperThicknessLimit)
      {
        thickness = thickness + 5 ;
        lcd.setCursor(14, 0);
        lcd.print("  ");
      }
      else
      {
        alarm_();
      }
      pause_ = true;
    }
    else if (customKey == 'C')
    {
      if (thickness > lowerThicknessLimit)
      {
        thickness = thickness - 5 ;
        lcd.setCursor(14, 0);
        lcd.print("  ");
      }
      else
      {
        alarm_();
      }
      pause_ = true;
    }

    if ( pause_ == false)
    {
      unsigned long stopTimer = millis() - startTimer;
      if (stopTimer > 2000)
      {
        break;
      }
    }
  }
  pause_ = false;
  //  }
}

void printPleaseWait()
{
  lcd.clear();
  lcd.setCursor(2, 1);
  lcd.print("Please Wait...");
}
