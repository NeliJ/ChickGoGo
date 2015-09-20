#include <IRremote.h>


const int ULTRASONIC_TRIGGER = 2;
const int ULTRASONIC_ECHO = 3;
unsigned long ULTRASONIC_LAST = 0;

// Specify the ULN2003 IN1,IN2,IN3,IN4 mapping to Arduino 8,9,10,11
const int MOTOR1PIN1 = 8;
const int MOTOR1PIN2 = 9;
const int MOTOR1PIN3 = 10;
const int MOTOR1PIN4 = 11;

// Specify the ULN2003 IN1,IN2,IN3,IN4 mapping to Arduino 4,5,6,7
const int MOTOR2PIN1 = 4;
const int MOTOR2PIN2 = 5;
const int MOTOR2PIN3 = 6;
const int MOTOR2PIN4 = 7;

// Specify the delay between each step. max speed: 1
const int MOTORSPEED = 1; 

// The infrared signal from your infrared transmitter.
// Modify it to meet your infrared transmitter's signal value.
const long SIGNAL_FORWARD = 0x00FF629D;
const long SIGNAL_BACKWARD = 0x00FF02FD;
const long SIGNAL_LEFT = 0x00FF22DD;
const long SIGNAL_RIGHT = 0x00FFC23D;
const long SIGNAL_POWER= 0x00FFA25D;
const long SIGNAL_INVALID = 0x00000000;
const int SIGNALS_LENGTH = 6;

// define the available values which handled by our codes
const long SIGNALS[SIGNALS_LENGTH] = { 
  SIGNAL_FORWARD, SIGNAL_BACKWARD, SIGNAL_LEFT, SIGNAL_RIGHT, SIGNAL_POWER
};
// define the pin number which receive Infrared signal
const int IR_PIN = 12; 
// cache the previous signal we received.
long _CurSignal = 0x00000000;
// if power is off, we don't handle infrared signal. 0: off 1: on
int _PowerOn = 0;

unsigned long _Last = millis();

IRrecv _Irrecv(IR_PIN);
decode_results _Results;

// use this value to trace the current step for stepper
int _CurPhase = 0; 

void setup() {
  pinMode(ULTRASONIC_TRIGGER, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(MOTOR1PIN1, OUTPUT);
  pinMode(MOTOR1PIN2, OUTPUT);
  pinMode(MOTOR1PIN3, OUTPUT);
  pinMode(MOTOR1PIN4, OUTPUT);
  pinMode(MOTOR2PIN1, OUTPUT);
  pinMode(MOTOR2PIN2, OUTPUT);
  pinMode(MOTOR2PIN3, OUTPUT);
  pinMode(MOTOR2PIN4, OUTPUT);
  
  pinMode(IR_PIN, INPUT);
  // Start the receiver  
  _Irrecv.enableIRIn(); 
  Serial.begin(9600);
}

void handleSignal() {
  if (_Irrecv.decode(&_Results)) {
    // Handle next infrared signal with interval greater than 250 ms
    if (millis() - _Last > 250) {
      long signal = dumpSignal(&_Results);
      if (isValidSignal(signal)) {
        _CurSignal = signal;
      }
    }
    _Last = millis();
    _Irrecv.resume();  // Receive the next value    
  }
}


float getDistance() {
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  
  float distance = pulseIn(ULTRASONIC_ECHO, HIGH);
  distance *= 0.01657;
  
  Serial.print(distance);
  Serial.println(" cm");
  //delay(50);
  return distance;
}


void loop() {
  
  handleSignal();  // check if recieve next signal
  
  // if power button is clicked, toggle power status
  // Set the robot going forward if power is on
  if (SIGNAL_POWER == _CurSignal) {  
    _PowerOn = !_PowerOn;
    _CurSignal = SIGNAL_FORWARD;
  }
  
  if (_PowerOn) {
    // check distance every 1/4 second
    if (millis()- ULTRASONIC_LAST > 50) {
      float distance = getDistance();  // check distance from the front object
      
      if (distance >= 25.0) {
        _CurSignal = SIGNAL_FORWARD;
        
      } else {
        _CurSignal = SIGNAL_LEFT;
      }
      
      ULTRASONIC_LAST = millis();
    }
    
    // Drive motor in sequences between step1 to step8
    _CurPhase = _CurPhase+1 < 8 ?  _CurPhase+1 : 0;
    switch(_CurSignal) {
      case SIGNAL_FORWARD:
        forward(_CurPhase);
        break;
      case SIGNAL_LEFT:
        left(_CurPhase);
        break;
      case SIGNAL_BACKWARD:
        backward(_CurPhase);
        break;
      case SIGNAL_RIGHT:
        right(_CurPhase);
        break;        
      default:
        break;
    }
  }
}

bool isValidSignal(long signal) {
  bool isValid = false;
  for (int i=0; i<SIGNALS_LENGTH;i++) {
    if (signal == SIGNALS[i]) {
      isValid = true;
      break;
    }
  }   
  return isValid;
}

// dump the infrared signal and print out it value
long dumpSignal(decode_results *results) {
  long targetSignal = SIGNAL_INVALID;
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
    Serial.println("Could not decode message");
  } else {
    targetSignal = results->value;
    Serial.print("key: ");
    Serial.println(results->value, HEX);
  }
  return targetSignal;
}

// We place the two stepper in opposite direction, so we need to
// drive it in the opposite direction too
void backward(int phase) {
  switch(phase) {
    case 0:
      step8(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step1(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 1:
      step7(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step2(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 2:
      step6(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step3(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 3:
      step5(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step4(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 4:
      step4(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step5(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 5:
      step3(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step6(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 6:
      step2(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step7(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 7:
      step1(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step8(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    default:
      break;
  }
  delay(MOTORSPEED);
}

void forward(int phase) {
  switch(phase) {
    case 0:
      step1(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step8(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 1:
      step2(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step7(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 2:
      step3(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step6(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 3:
      step4(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step5(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 4:
      step5(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step4(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 5:
      step6(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step3(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 6:
      step7(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step2(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 7:
      step8(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step1(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    default:
      break;
  }
  delay(MOTORSPEED);
}

// One stepper go clockwise and the other one go counter clockwise
void left(int phase) {
  switch(phase) {
    case 0:
      step1(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step1(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 1:
      step2(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step2(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 2:
      step3(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step3(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 3:
      step4(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step4(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 4:
      step5(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step5(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 5:
      step6(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step6(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 6:
      step7(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step7(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 7:
      step8(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step8(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    default:
      break;
  }
  delay(MOTORSPEED);
}

void right(int phase) {
  switch(phase) {
    case 0:
      step8(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step8(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 1:
      step7(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step7(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 2:
      step6(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step6(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 3:
      step5(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step5(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 4:
      step4(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step4(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 5:
      step3(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step3(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 6:
      step2(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);
      step2(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    case 7:
      step1(MOTOR1PIN1, MOTOR1PIN2, MOTOR1PIN3, MOTOR1PIN4);    
      step1(MOTOR2PIN1, MOTOR2PIN2, MOTOR2PIN3, MOTOR2PIN4);
      break;
    default:
      break;
  }
  delay(MOTORSPEED);
}

// specify HIGH/LOW according to 4 phase specification. check Fig 1.
void step1(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
}

void step2(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
}

void step3(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, LOW);
}

void step4(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, LOW);
}

void step5(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, LOW);
}


void step6(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, HIGH);
}

void step7(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, HIGH);
}


void step8(int pin1, int pin2, int pin3, int pin4) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  digitalWrite(pin3, LOW);
  digitalWrite(pin4, HIGH);
}

