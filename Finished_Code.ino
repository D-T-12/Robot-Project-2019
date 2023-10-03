/*
 * Lancaster University Robot Project
 * Final Code - Group U - 18/03/19
 * Author - Benedict Draper-Turner
 */
 
#include <Servo.h>

int count = 0;
int flag = 0;
int P = 0;
int I = 0;
int D = 0;
float Kp = 1;
float Ki = 0;
float Kd = 5;
int error;
int adjustedError;
int previousError;
int PIDval;
int senseInput;
int setPoint = 0;
int motorSpeed = 75;
int servoAngle = 90;

void calculatePID();
void calculateError();
void motorControl();

Servo theServo;

// song setup

int speakerPin = 9;

int length = 42; // the number of notes
char notes[] = "dfg dfg dfgbbaafg efgedcefgedccefgedcbab  "; // a space represents a rest
int beats[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 4, 2, 2, 2, 2, 4, 4, 2, 2, 2, 2, 4, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 4};
int tempo = 150;

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1150, 1014, 956 };

  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}
//song setup end 

void setup() {
  
  EIMSK = _BV(INT0);                        //eneables interrupts for pin 2
  EICRA = _BV(~ISC01) | _BV(ISC00);         //sets iterupt to occur on any logic change on pin 2 (switch) 
  DDRD = _BV(PD7) | _BV(PD6) | _BV(PD5);    //sets pin 7(arm motor) , 6 (motor) and 5 (motor) as outputs
  DDRC = _BV(~PC3);                                               //sets A3 (light sensing) to an input       
  DDRB = _BV(PB3);                                                //sets pin 11 (servo) as an output 
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);   //sets up fast PWM mode
  TCCR0B = _BV(CS01) | _BV(CS00) | _BV(~CS02);                    //sets prescaler of 64
  OCR0A = 0;                                                      //sets motor control to 0 to start
  OCR0B = 0;                                                      
  
  theServo.attach(11);     //attatches pin 11 to the servo
  theServo.write(0);       //sets the servo to start position
  setPoint = analogRead(3); //sets the setPoint when the robot is switched on

  Serial.begin(9600);
  
  //song setup
  pinMode(speakerPin, OUTPUT);
 // song setup end
 
}

void loop() {

  //Serial.print("1");
  //Serial.println();
  
  if(count == 0){

     calculatePID();
     calculateError();
     motorControl();

     //Serial.print("2");
     //Serial.println();
      Serial.print(setPoint);
      Serial.print("       ");
      Serial.print(error);
      Serial.print("       ");
      Serial.print(OCR0A);
      Serial.print("       ");
      Serial.print(OCR0B);
      Serial.println();
      
    }
    if(count == 1){

     OCR0A = 0;                                                      
     OCR0B = 0;
     motorSpeed = 0;
    theServo.write(servoAngle);          //moves arm into position 
    delay(1000);
      if(flag == 0){
        PORTD = _BV(PORTD7);      //posts letter 
      }
    delay(720);
    PORTD = _BV(~PORTD7);
    count++;
         
       Serial.print("3");
       Serial.println();
      
      }
      if(count >= 2){
        
        //Serial.print("4");
        //Serial.println();

    //song loop 
          for (int i = 0; i < length; i++) {
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }

    // pause between notes
    delay(tempo / 2); 
  }
   //song loop end     
        }

}


void calculatePID(){                //calculates the PID error value
  
  P = error;
  I = I + error;
  D = error - previousError;
  PIDval = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error; 
  
}

void calculateError(){              //calculates the current error from the centre of the line
                                    //and converts it into a value the PID can use
 senseInput = analogRead(3);
 error = setPoint - senseInput;
 constrain(PIDval, -motorSpeed, motorSpeed);
 error = map(error, -setPoint, setPoint, -motorSpeed, motorSpeed); 
 
  }

void motorControl(){            //adjusts motor speed based on error
    
  OCR0A = motorSpeed + PIDval;
  OCR0B = motorSpeed - PIDval;
  
  }

  
ISR(INT0_vect){     //Interrupt service routine for when robot hits wall

  count++;          //Increases count by 1
  cli();

}
