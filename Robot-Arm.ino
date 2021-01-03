#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* Joystick Pins */
#define SW_PIN_0 2
#define X_PIN_0 0
#define Y_PIN_0 1

#define SW_PIN_1 3
#define X_PIN_1 2
#define Y_PIN_1 3

/* Servo Values */
#define SERVOMIN  120
#define SERVOMAX  610

#define ROOF 0

class Servo {
    Adafruit_PWMServoDriver driver;
    uint8_t number;
    uint16_t servoMaxPosition = SERVOMAX;
    uint16_t servoMinPosition = SERVOMIN;
    uint16_t servoPosition = (servoMinPosition + servoMaxPosition) / 2;
  public:
    Servo(const Adafruit_PWMServoDriver aDriver, const uint8_t aNumber) {
      driver = aDriver;
      number = aNumber;
    }

    uint16_t getPosition() {
      return servoPosition;
    }

    void setPosition(const uint16_t aPosition) {
      if (aPosition < servoMinPosition) {
        servoPosition = servoMinPosition;
      } else if (aPosition > servoMaxPosition) {
        servoPosition = servoMaxPosition;
      } else {
        servoPosition = aPosition;
        driver.setPWM(number, 0, servoPosition);
      }
      //Serial.println(servoPosition);
    }

    void move(const int16_t aStep) {
      uint16_t tPosition = servoPosition;
      tPosition += aStep;
      setPosition(tPosition);
    }
};

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

uint16_t yAxisBorders[8] { 895, 770, 635, 542, 482, 355, 250, 125 };
uint16_t xAxisBorders[8] { 895, 767, 659, 531, 470, 348, 245, 120 };
Servo servo0(servoDriver, 15);
Servo servo1(servoDriver, 14);
Servo servo2(servoDriver, 13);
Servo servo3(servoDriver, 12);

uint16_t servoMeanPosition = (SERVOMAX + SERVOMIN) / 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(SW_PIN_0, INPUT);
  digitalWrite(SW_PIN_0, HIGH);
  Serial.begin(9600);

  servoDriver.begin();
  servoDriver.setPWMFreq(60);
  servo0.setPosition(servoMeanPosition);
  servo1.setPosition(servoMeanPosition);
  servo2.setPosition(servoMeanPosition);
  servo3.setPosition(servoMeanPosition);
  delay(100);
}

void loop() {
  const int8_t rotate = moveOrder(yAxisBorders, Y_PIN_0);
  const int8_t upperArm = moveOrder(xAxisBorders, X_PIN_0);
  const int8_t foreArm = moveOrder(xAxisBorders, X_PIN_1);
  const int8_t hand = moveOrder(yAxisBorders, Y_PIN_1);
  if (rotate != 0 || upperArm != 0 || foreArm != 0 || hand != 0) {
    if (rotate != 0) {
      const int8_t dirRotate = rotate < 0 ? -2 : 2;
      servo0.move(dirRotate);
    }
    if (upperArm != 0) {
      const int8_t dirUpperArm = upperArm < 0 ? -2 : 2;
      servo1.move(dirUpperArm);
    }
    if (foreArm != 0) {
      const int8_t dirForeArm = foreArm < 0 ? 2 : -2;
      servo2.move(dirForeArm);
    }
    if (hand != 0) {
      const int8_t dirHand = hand < 0 ? 2 : -2;
      servo3.move(dirHand);
    }
    uint8_t absUpperArm = abs(upperArm);
    uint8_t absRotate = abs(rotate);
    uint8_t absForeArm = abs(foreArm);
    uint8_t absHand = abs(hand);
    delay(max(absUpperArm, max(absRotate, max(absForeArm, absHand))));
    //delay(absUpperArm > absRotate ? absUpperArm : absRotate);
  } else {
    delay(100);
  }

}

/*
 * 
 */
int8_t moveOrder(const uint16_t axisBorders[8], const uint8_t pin) {
  uint16_t axisValue = analogRead(pin);
  if (axisValue >= axisBorders[0])
    return 1;
  else if (axisValue >= axisBorders[1])
    return 2;
  else if (axisValue >= axisBorders[2])
    return 10;
  else if (axisValue >= axisBorders[3])
    return 20;
  else if (axisValue >= axisBorders[4])
    return 0;
  else if (axisValue >= axisBorders[5])
    return -20;
  else if (axisValue >= axisBorders[6])
    return -10;
  else if (axisValue >= axisBorders[7])
    return -2;
  else
    return -1;
}
