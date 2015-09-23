#include <IRremote.h>

// Specify UltraSonic sensor pins
const int ULTRASONIC_TRIGGER = 2;
const int ULTRASONIC_ECHO = 3;
unsigned long _UltrasonicLast = 0;

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
const int  SIGNALS_LENGTH = 5;

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

unsigned long _InfraredLast = millis();

IRrecv _Irrecv(IR_PIN);
decode_results _Results;

// use this value to trace the current step for stepper
int _CurStep = 0; 

//http://42bots.com/tutorials/28byj-48-stepper-motor-with-uln2003-driver-and-arduino-uno/
// 1 rev : 4076 step
const unsigned long STEPS_PER_REV = 4076;
const unsigned long SHAKE_HEAD_DEGREE = 13;
const float COLLISION_WARN_DISTANCE = 8.0;
const float COLLISION_SAFE_DISTANCE = 16.0;
int _CurState = 0;
const int STATE_WALK = 0;
const int STATE_CHECK = 1;
const int SAFE_COUNTER = 4;

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

void loop() {
  long signal = getIRSignal();  // check if recieve next signal
  
  // if power button is clicked, toggle power status
  // Set the robot going forward if power is on
  if (SIGNAL_POWER == signal) {  
    _PowerOn = !_PowerOn;    
    _CurSignal = SIGNAL_FORWARD;
  } else if (SIGNAL_INVALID != signal) {
    _CurSignal = signal;
  }

  if (_PowerOn) {
    // check distance every 50 ms
    if (millis()- _UltrasonicLast > 50) {
      float distance = getDistance(); // distance from the front object
      if (distance < COLLISION_WARN_DISTANCE) {
        _CurState = STATE_CHECK;
      } else {
        _CurState = STATE_WALK;
      }
      _UltrasonicLast = millis();
    }

    if (STATE_WALK == _CurState) {
      _CurStep = _CurStep+1 < 8 ?  _CurStep+1 : 0;
      walk(_CurSignal, _CurStep);
    } else {
      evade();
      _CurState = STATE_WALK;  
      _CurSignal = SIGNAL_FORWARD;
    }
  }
}

// handle walking when collision is not happened
void walk(long signal, int aStep) {
  // Drive motor in sequences between step1 to step8
  switch(signal) {
    case SIGNAL_FORWARD:
      forward(aStep);
      break;
    case SIGNAL_LEFT:
      left(aStep);
      break;
    case SIGNAL_BACKWARD:
      backward(aStep);
      break;
    case SIGNAL_RIGHT:
      right(aStep);
      break;        
    default:
      break;
  }
}

// handle movement when collision is happend
void evade() { 
  int aStep = 0;
  // turn left degree of SHAKE_HEAD_DEGREE
  long steps = stepsFromDegree(SHAKE_HEAD_DEGREE);
  while (steps-- >=0) {
    aStep = aStep+1 < 8 ?  aStep+1 : 0;
    left(aStep);
  }
  float leftDiffDistance = getDistance();

  // turn right degree of SHAKE_HEAD_DEGREE. 
  // 1*SHAKE_HEAD_DEGREE is for turning back to initial position
  steps = stepsFromDegree(2 * SHAKE_HEAD_DEGREE);
  while (steps-- >=0) {
    aStep = aStep+1 < 8 ?  aStep+1 : 0;
    right(aStep);
  }
  float rightDiffDistance = getDistance();  
  
  unsigned long lastDetect = millis();
  int safeCounter = 0;
  while(millis() - lastDetect  < 20000) {
    long signal = getIRSignal();  // check if recieve next signal
    if (SIGNAL_POWER == signal) {
      break;
    }
    
    // left can reach more. so we go left
    steps = stepsFromDegree(SHAKE_HEAD_DEGREE);
    while (steps-- >=0) {      
      aStep = aStep+1 < 8 ?  aStep+1 : 0;
      if (leftDiffDistance > rightDiffDistance) {
        left(aStep);
      } else {
        right(aStep);
      }
    }
    
    float curDist = getDistance();
    if (curDist >= COLLISION_SAFE_DISTANCE) {
      safeCounter++;
      if (SAFE_COUNTER == safeCounter) {
//        // it's really safe. turn back and go
//        steps = stepsFromDegree((safeCounter / 2) * SHAKE_HEAD_DEGREE);
//        while (steps-- >=0) {      
//            aStep = aStep+1 < 8 ?  aStep+1 : 0;
//            if (leftDiffDistance > rightDiffDistance) {
//              right(aStep);
//            } else {
//              left(aStep);
//            }
//        }
        break;
      }
    }
  }
}

/*
 * Handle signal from infrared sensor
 */
long getIRSignal() {
  long curSignal = SIGNAL_INVALID;
  if (_Irrecv.decode(&_Results)) {
    // Handle next infrared signal with interval greater than 250 ms
    if (millis() - _InfraredLast > 250) {
      long signal = dumpSignal(&_Results);
      if (isValidIRSignal(signal)) {
        curSignal = signal;
      }
    }
    _InfraredLast = millis();
    _Irrecv.resume();  // Receive the next value    
  }
  return curSignal;
}

bool isValidIRSignal(long signal) {
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

/*
 * Detect distance from ultrasonic sensor
 */
float getDistance() {
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  
  float distance = pulseIn(ULTRASONIC_ECHO, HIGH);
  distance *= 0.01657;
  
//  Serial.print(distance);
//  Serial.println(" cm");
  //delay(50);
  return distance;
}

float stepsFromDegree(float degree) {
  return 4076.0 * (degree/360.0);
}

/*
 * Handle 28BYJ-48 Stepper motor
 */
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

