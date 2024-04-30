
//#include <ESP32Encoder.h>
#include <Keypad.h>
#include <Wire.h>
#include <ESP32Encoder.h>

ESP32Encoder encoder1;
ESP32Encoder encoder2;

int16_t a = 0;
int16_t b = 0;


#define I2C_SLAVE_ADDR 0x08  // 8 in hexadecimal

// L298 motor driver pin definitions

#define enA 33  // enableA command line
#define enB 25  // enableB command line
#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction



// setting up the PWM properties used for the motors

const int freq = 2000;
const int ledChannela = 0;  // assumed as the channel used for the left motor
const int ledChannelb = 1;  // assumed as the channel used for the righteft motor
const int resolution = 8;   // 8-bit PWM signal

int servoPin = 13;  //the servo is attached to IO_13 on the ESP32


// setting up the PWM properties of the servo
// as an aside, the control signal of the SG90 servo is a PWM signal with a period of 20ms (50Hz) and the pulse duration has to be between 1ms to 2ms i.e. 5% duty cycle for the minimum angle of 0, and 10% duty cycle for the maximum angle of 180
// it is not recommended that you change any of the four values below

int dutyCycle = 5;

const int servoFrequency = 50;   // 50Hz signal
const int servoChannel = 2;      // channels 0 and 1 are used for the two motors on your EEEBot
const int servoResolution = 12;  // 12-bit PWM signal


void setup() {

  ESP32Encoder::useInternalWeakPullResistors = DOWN;
  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder1.attachHalfQuad(34, 35);
  encoder2.attachHalfQuad(36, 39);

  encoder1.setCount(0);
  encoder2.setCount(0);

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  // configure LED PWM functionalities

  ledcSetup(ledChannela, freq, resolution);
  ledcSetup(ledChannelb, freq, resolution);
  ledcSetup(servoChannel, servoFrequency, servoResolution);  //servo setup on PWM channel 2, 50Hz, 12-bit (0-4095)


  // attach the channel to the GPIO to be controlled

  ledcAttachPin(enA, ledChannela);
  ledcAttachPin(enB, ledChannelb);
  ledcAttachPin(servoPin, servoChannel);

  Wire.begin(I2C_SLAVE_ADDR);    // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
  Wire.onRequest(requestEvent);  // create a receive event
  Wire.onReceive(onReceive);     // receive event


  Serial.begin(115200);             // start serial for the output
  Serial.println("ESP32 Running");  // sanity check
}


void onReceive(int howMany) {

  int16_t leftMotor_speed = 0;
  int16_t rightMotor_speed = 0;
  int16_t servoAngle = 0;
  int16_t flag = 0;

  uint8_t leftMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of y (one byte)
  uint8_t leftMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of y (one byte)

  uint8_t rightMotor_speed16_9 = Wire.read();  // receive bits 16 to 9 of z (one byte)
  uint8_t rightMotor_speed8_1 = Wire.read();   // receive bits 8 to 1 of z (one byte)

  uint8_t servoAngle16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t servoAngle8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)

  uint8_t flag16_9 = Wire.read();  // receive bits 16 to 9 of x (one byte)
  uint8_t flag8_1 = Wire.read();   // receive bits 8 to 1 of x (one byte)


  leftMotor_speed = (leftMotor_speed16_9 << 8) | leftMotor_speed8_1;     // combine the two bytes into a 16 bit number
  rightMotor_speed = (rightMotor_speed16_9 << 8) | rightMotor_speed8_1;  // combine the two bytes into a 16 bit number
  servoAngle = (servoAngle16_9 << 8) | servoAngle8_1;                    // combine the two bytes into a 16 bit number
  flag = (flag16_9 << 8) | flag8_1;     


  Serial.println("Left Motor: ");
  Serial.println(leftMotor_speed);
  Serial.println("\n");
  Serial.println("Right Motor: ");
  Serial.println(rightMotor_speed);
  Serial.println("\n");
  Serial.println("Servo: ");
  Serial.println(servoAngle);
  Serial.println("flag: ");
  Serial.println(flag);

  Servo_Setup(servoAngle);
  Motor_Setup(leftMotor_speed, rightMotor_speed, flag);


  delay(1000);
}


void emptyBuffer(void) {

  Serial.println("Error: I2C Byte Size Mismatch");
  while (Wire.available()) {
    Wire.read();
  }
}

void Servo_Setup(int servoAngle) {

  dutyCycle = map((constrain(servoAngle, 0, 180)), 0, 180, 205, 410);  // contrain() limits the minimum and maximum values to 0 and 180 respectively, map() proportionally scales values between 0 and 180 to values between 205 (5% duty cycle) and 410 (10% duty cycle)
  ledcWrite(servoChannel, dutyCycle);                                  // write the control signal to the PWM
}


void Motor_Setup(int leftMotor_speed, int rightMotor_speed, int flag) {
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);

  ledcWrite(ledChannela, abs(leftMotor_speed));
  ledcWrite(ledChannelb, abs(rightMotor_speed));

  // if the speed value is negative, run the motor backwards
  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);
  }
  if (flag = 1) {
    encoder1 = 0;
    encoder2 = 0;
  }
  else {}
}

void loop() {
  a = encoder1.getCount();
  b = encoder2.getCount();

  // Serial.print("Encoder Count 1: ");
  // Serial.print(a);
  // Serial.print("\n");
  // Serial.print("Encoder Count 2: ");
  // Serial.println(b);

  delay(500);
}

// this function executes when data is requested from the slave device
void requestEvent() {
  Wire.write((byte)((a & 0x0000FF00) >> 8));  // first byte of enc1Count, containing bits 16 to 9
  Wire.write((byte)(a & 0x000000FF));         // second byte of enc1Count, containing the 8 LSB - bits 8 to 1

  //Wire.write((byte)((enc2Count & 0xFF000000) >> 24)); // bits 32 to 25 of enc2Count
  //Wire.write((byte)((enc2Count & 0x00FF0000) >> 16)); // bits 24 to 17 of enc2Count
  Wire.write((byte)((b & 0x0000FF00) >> 8));  // first byte of enc2Count, containing bits 16 to 9
  Wire.write((byte)(b & 0x000000FF));
}
