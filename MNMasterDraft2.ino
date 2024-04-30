#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal.h>

#define I2C_SLAVE_ADDR 0x08  // I2C slave address

const int rs = 19, en = 23, d4 = 18, d5 = 17, d6 = 16, d7 = 15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
String keySequence = "";

const byte ROWS = 4;
const byte COLS = 3;

char hexaKeys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};

byte rowPins[ROWS] = { 32, 33, 25, 26 };
byte colPins[COLS] = { 27, 14, 12 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

const int L = 35;
char keyArray[L];
int ArrayIndex = 0;
int cursorColumn = 0;

int encoderRes = 45;
float circumference = 0.38;
float distance = 0;

int rightMotor_speed = 0;
int leftMotor_speed = 0;
int servoAngle = 75;

int leftTargetCounts = 80;  //CHANGE THESE
int rightTargetCounts = 80;

int enc1Count;
int enc2Count;
int flag = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);
  // lcd.print("Command:");
  //   lcd.setCursor(cursorColumn, 0);
  //   lcd.print(key);
  //   cursorColumn++;
}

void ReceiveData() {


  Wire.requestFrom(0x08, 4);
  // two 16-bit integer values are requested from the slave
  int16_t a = 0;
  int16_t b = 0;
  //uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t b16_9 = Wire.read();  // receive bits 16 to 9 of b (one byte)
  uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  a = (a16_9 << 8) | a8_1;  // combine the two bytes into a 16 bit number
  b = (b16_9 << 8) | b8_1;  // combine the two bytes into a 16 bit number

  enc1Count = a;
  enc2Count = b;
}
// Serial.println("encoder 1:");
// Serial.println(a);
// Serial.println("\n");
// Serial.println("encoder 2: ");
// Serial.println(b);

void transmitData() {
  // Transmit data over I2C
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));  // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));         // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));         // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));        // first byte of y, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));               // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((flag & 0x0000FF00) >> 8));              // first byte of y, containing bits 16 to 9
  Wire.write((byte)(flag & 0x000000FF));                     // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();                                    // stop transmitting
  delay(1000);
}

void caluclateDistance() {
  ReceiveData();

  distance = ((enc1Count + enc2Count) / 2) * (encoderRes / circumference);
  Serial.println("Distance: ");
  Serial.println(distance);
}

void loop() {


  char key = customKeypad.getKey();  // Read the pressed key

  if (key != NO_KEY) {  // Check if a key is pressed
    lcd.setCursor(cursorColumn, 0);
    lcd.print(key);
    cursorColumn++;

    if (ArrayIndex < L - 1) {        // Check if there's space in the array
      keyArray[ArrayIndex++] = key;  // Add the key to the array and increment the index

      // Print the contents of the array
      Serial.print("Key array: ");
      for (int i = 0; i < ArrayIndex; i++) {
        Serial.print(keyArray[i]);
      }
      Serial.println();
    } else {
      Serial.println("Array full");
    }

    Serial.print("Key pressed: ");
    Serial.println(key);
  }


  if (key == '#') {  // Check if '#' key is pressed
    // Clear the array
    ArrayIndex = 0;
    Serial.println("Array Cleared");
    lcd.clear();  // Clears LCD display
    cursorColumn = 0;
  }

  if (key == '*') {
    for (int j = 0; j < ArrayIndex; j++) {  // Go through each character
      char keyChar = keyArray[j];
      Serial.println("Breaking down array");
      lcd.clear();
      cursorColumn = 0;




      switch (keyChar) {
        case '2':
          distance = 0;
          while (distance < 9) {
            rightMotor_speed = leftMotor_speed = 250;
            caluclateDistance();
            transmitData();
            Serial.println("Forward");
          }
          // flag = 1;
          rightMotor_speed = leftMotor_speed = 0;
          distance = 0;
          break;

        case '8':
          rightMotor_speed = leftMotor_speed = -250;
          transmitData();
          Serial.println("Reversing");

          break;

        case '4':
          servoAngle = 0;
          transmitData();
          Serial.println("Turn left");

          break;

        case '6':
          servoAngle = 180;
          transmitData();
          Serial.println("Turn right");

          break;

        case '5':
          servoAngle = 75;
          transmitData();
          Serial.println("Set servo straight");

          break;

        case '0':
          rightMotor_speed = leftMotor_speed = 0;
          transmitData();
          Serial.println("Stop!");

          break;

        case '*':
          servoAngle = 75;
          leftMotor_speed = rightMotor_speed = 0;
          transmitData();
          Serial.println("Command chain complete");
          break;

        case '1, 3, 7, 9':
          Serial.println("No command assigned to key");
          break;
      }
    }
  }
}


// void onReceive(int howMany) {
//   if (howMany != 4) {
//     emptyBuffer();
//     return;
//   }
//   // two 16-bit integer values are requested from the slave
//   int16_t a = 0;
//   int16_t b = 0;
//   uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
//   uint8_t a16_9 = Wire.read();                                  // receive bits 16 to 9 of a (one byte)
//   uint8_t a8_1 = Wire.read();                                   // receive bits 8 to 1 of a (one byte)
//   uint8_t b16_9 = Wire.read();                                  // receive bits 16 to 9 of b (one byte)
//   uint8_t b8_1 = Wire.read();                                   // receive bits 8 to 1 of b (one byte)

//   a = (a16_9 << 8) | a8_1;  // combine the two bytes into a 16 bit number
//   b = (b16_9 << 8) | b8_1;  // combine the two bytes into a 16 bit number

//   int enc1Count = a;
//   int enc2Count = b;

//   Serial.println("encoder 1:");
//   Serial.println(a);
//   Serial.println("\n");
//   Serial.println("encoder 2: ");
//   Serial.println(b);
// }

// void emptyBuffer(void) {
//   Serial.println("Error");
//   while (Wire.available()) {
//     Wire.read();
//   }
//}
//Serial.print(leftMotor_speed);
//Serial.print("\t");
//Serial.print(rightMotor_speed);
//Serial.print("\t");
//Serial.println(servoAngle);
