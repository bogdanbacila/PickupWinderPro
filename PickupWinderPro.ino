//This code is to control the speed of a DC motor by a potentiometer using l298n driver
//We read the value from the analog input, calibrate it then inject to the module
//Refer to Surtrtech youtube channel for more information

#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.
#include <Wire.h>
#include <hd44780.h> // include hd44780 library header file
#include <hd44780ioClass/hd44780_I2Cexp.h> // i/o expander/backpack class
#include <Stepper.h>

// --- Declare pins ---
byte mot1Pin = 8; //Declaring where our module is wired
byte mot2Pin = 9;
byte spindleCtlPin = 3;  //Speed potentiometer - Don't forget this is a PWM DI/DO
byte counterPin = 2; // IR sensor pin - Should be an interrupt pin 
byte dirPin = 4;
byte rightPin = 5;
byte leftPin = 6;
byte resetPin = 7;
byte homePin = A1;


// --- Declare variables ---
byte dirBtnValue = HIGH;           // the current reading from the input pin
byte previousDirBtnVal = LOW;    // the previous reading from the input pin
byte resetBtnValue = HIGH;           // the current reading from the reset pin
byte previousResBtnVal = LOW;    // the previous reading from the counter reset pin
byte homeBtnValue = HIGH;           // the current reading from the reset pin
byte previousHomeBtnVal = LOW;    // the previous reading from the counter reset pin

byte leftBtnValue = HIGH;           // the current reading from the input pin
byte rightBtnValue = HIGH;           // the current reading from the reset pin

bool spindleDir = 0; 
int8_t xDir = 1; 
int xMultiplier = 5 ;

int pickupWidth = 150;
int prevPotVal = 0;
int xPos = 0;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

volatile int counter = 0;
volatile int interruptFlag = 0;

hd44780_I2Cexp lcd; // auto detect backpack and pin mappings
Stepper xAxis(512, 10, 11, 12, 13); //steps per revolution -> need to recheck this - and the motor connections


void setup() {
  
  //--- Set PWM frequency for D3 & D11 ---

  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  // --- define variables ---
  pinMode(mot1Pin, OUTPUT);
  pinMode(mot2Pin, OUTPUT);  
  pinMode(spindleCtlPin, OUTPUT);
  pinMode(dirPin, INPUT);
  pinMode(resetPin, INPUT);
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(homePin, INPUT);
  pinMode(counterPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(counterPin), rotationISR, RISING);

  // ---- setup LCD --- 
  int status;
  // initialize LCD with number of columns and rows: 
  // hd44780 returns a status from begin() that can be used
  // to determine if initalization failed.
  // the actual status codes are defined in <hd44780.h>
  // See the values RV_XXXX
  //
  // looking at the return status from begin() is optional
  // it is being done here to provide feedback should there be an issue
  //
  // note:
  //  begin() will automatically turn on the backlight
  //
  status = lcd.begin(16, 2);
  if(status) // non zero status means it was unsuccesful
  {
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }
  // LCD initalization was successful, the backlight should be on now
  
  Serial.begin(9600);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PickupWinder Pro");
  delay(1000);
  lcd.clear();
  lcd.print("Count Dir XPos");
  updateLCD_counter();
  updateLCD_direction(spindleDir);
  updateLCD_xPos();
  

  //--
  xAxis.setSpeed(60);
  
  
}


void loop() {

  dirBtnValue = digitalRead(dirPin);
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (dirBtnValue == HIGH && previousDirBtnVal == LOW && millis() - time > debounce) {
    spindleDir = !spindleDir;
    updateLCD_direction(spindleDir);
    time = millis();    
  }
  previousDirBtnVal = dirBtnValue;

  resetBtnValue = digitalRead(resetPin);
  if(resetBtnValue == HIGH && previousResBtnVal == LOW && millis() - time > debounce ){
    
    counter = 0; 
    updateLCD_counter();
    
    xPos = 0;
    xDir = 1;
    updateLCD_xPos();

    
  }
  previousResBtnVal = resetBtnValue;

  homeBtnValue = digitalRead(homePin);
  if(homeBtnValue == HIGH && previousHomeBtnVal == LOW && millis() - time > debounce ){
    pickupWidth = xPos;
    xDir = 1; 
    moveXAxis(-xPos);
    updateLCD_xPos();

  }
  previousHomeBtnVal = homeBtnValue;

  leftBtnValue = digitalRead(leftPin);
  if(leftBtnValue == HIGH  ){
    moveXAxis(-5);
    updateLCD_xPos();

  }
 

   rightBtnValue = digitalRead(rightPin);
  if(rightBtnValue == HIGH  ){
    moveXAxis(5);
    updateLCD_xPos();

  }
 
  controlMotor(analogRead(A0), spindleDir);
  
//  Serial.println(analogRead(A0));
  // update LCD with counter value from ISR
  // move X Axis
  if(interruptFlag == 1){
    
    updateLCD_counter();
    updateLCD_xPos();

    
    //moveXAxis(getXSteps(((xPos/pickupWidth)%2 == 0), xMultiplier));
    moveXAxis(xMultiplier * xDir);
    xDir = toggleDirection(xPos, pickupWidth);

    
    interruptFlag = 0;
  }
}

void controlMotor(int potValue, bool spindleDir){
  int motSpeed = map(potValue, 0, 1023, 255, 137);
  
//  Serial.print(potValue);
//  Serial.print(", ");
//  Serial.println(motSpeed);
 
  
  if(potValue != prevPotVal){
    switch (potValue){
    case 1020 ... 1023:
      turnSpindle(spindleDir, 0); 
      break;  
    default:
      turnSpindle(spindleDir, motSpeed);
      break;
    }
  }
  prevPotVal = potValue; 
}

void turnSpindle(byte dir, char motSpeed){ //We create a function which control the direction and speed
  switch (dir) {
  case 0:
    digitalWrite(mot1Pin, LOW); //Switch between this HIGH and LOW to change direction
    digitalWrite(mot2Pin, HIGH);
    break;
  case 1: 
    digitalWrite(mot1Pin, HIGH); //Switch between this HIGH and LOW to change direction
    digitalWrite(mot2Pin, LOW);
    break;
  default: 
    digitalWrite(mot1Pin, LOW); //Switch between this HIGH and LOW to change direction
    digitalWrite(mot2Pin, HIGH);
    break;
  }
  
  analogWrite(spindleCtlPin, motSpeed);// Then inject it to our motor
}

void updateLCD_direction(byte spDir){
  switch (spDir) {
  case 0:
    lcd.setCursor(6,1);
    lcd.print("    ");
    lcd.setCursor(6,1);
    lcd.print("CW");
    break;
  case 1:
    lcd.setCursor(6,1);
    lcd.print("    ");
    lcd.setCursor(6,1);
    lcd.print("CCW");
    break;    
  }
}

void updateLCD_counter(){
  lcd.setCursor(0,1);
  lcd.print("      ");
  lcd.setCursor(0,1);
  lcd.print(counter);
}

void updateLCD_xPos(){
  lcd.setCursor(10,1);
  lcd.print("     ");
  lcd.setCursor(10,1);
  lcd.print(xPos);
}

void moveXAxis(int stepsToMove){
  xPos = xPos + stepsToMove;
  xAxis.step(stepsToMove);
}

int getXSteps(byte xDir, int xMultiplier){
  int stepsToMove = ((xDir == 0) ? xMultiplier * -1 : xMultiplier); 
  
  return stepsToMove;
}

byte toggleDirection(int xPos, int pickupWidth){
  if (xPos+xMultiplier > pickupWidth || xPos-xMultiplier < 0){
    return -xDir;
  }else {
    return xDir;
  }
}


void rotationISR() {  //ISR fro full rotation triggered by sensor
  counter++;
  interruptFlag = 1;
}
