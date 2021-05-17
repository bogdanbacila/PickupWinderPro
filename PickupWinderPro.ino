//This code is to control the speed of a DC motor by a potentiometer using l298n driver
//We read the value from the analog input, calibrate it then inject to the module
//Refer to Surtrtech youtube channel for more information

#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.


int mot1Pin = 8; //Declaring where our module is wired
int mot2Pin = 9;
int spindleCtlPin = 3;// Don't forget this is a PWM DI/DO
int dirPin = 4;
int counterPin = 2; // Should be an interrupt pin 

byte spindleDir = 1; 
int state = HIGH;
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

int potValue;
int speed1;
int speed2;
volatile int counter = 0;


void setup() {
  
//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  
  pinMode(mot1Pin, OUTPUT);
  pinMode(mot2Pin, OUTPUT);  
  pinMode(spindleCtlPin, OUTPUT);
  pinMode(dirPin, INPUT);
  pinMode(counterPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(counterPin), updateCounter, RISING);

  
  Serial.begin(9600);
}


void loop() {

  reading = digitalRead(dirPin);
 

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    spindleDir = !spindleDir;
    counter = 0;  //TODO: CHANGE THIS AFTER THE TEST TO A SEPARATE BUTTON 
    time = millis();    
  }


  previous = reading;

  potValue = analogRead(A0);
  speed1 = potValue * 0.2492668622; //We read thea analog value from the potentiometer and calibrate it. (1023 -> 255)
 
  speed2 = map(potValue, 1, 1023, 145, 255);

  //Serial.print(speed1);
  //Serial.print(", ");
  //Serial.println(speed2);
 


  switch (potValue){
    case 0:
      turnSpindle(spindleDir, 0); 
      break;  
    default:
      turnSpindle(spindleDir, speed2);
      break;
  }

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


void updateCounter() {  //ISr for updating the counter when the interrupt is triggered
  counter++;
  Serial.println(counter);
}
