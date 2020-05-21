#include <avr/io.h>

#define PWMPin 10

//unsigned int frq = 440; // 周波数
//float duty = 0.5; // 指定したいデューティ比
unsigned int frq = 16; // 周波数
float duty; // 指定したいデューティ比

byte incomingByte; // for incoming serial data
int dec;

void setup() {
  Serial.begin(115200);
  pinMode(PWMPin, OUTPUT);
}

void loop() {

  // モード指定
  TCCR1A = 0b00100001;
  TCCR1B = 0b00010010;

  // TOP値指定
  OCR1A = (unsigned int)(1000000 / frq);


  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    dec = (int)incomingByte;
    duty = float(dec)/255.0;

  // Duty比指定
    OCR1B = (unsigned int)(1000000 / frq * duty);
  }

  
}
