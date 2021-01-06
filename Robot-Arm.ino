#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*** Joystick Pins ***/
/* Joystick 0 for rotation and upper arm */
#define SW_PIN_0 2 // digital pin
#define X_PIN_0 0 // analog pin
#define Y_PIN_0 1 // analog pin
/* Joystick 1 for forearm and hand */
#define SW_PIN_1 3 // digital pin
#define X_PIN_1 2 // analog pin
#define Y_PIN_1 3 // analog pin

/* Servo Values */
#define SERVOMIN  120
#define SERVOMAX  610

/**
   Saves servo position and moves servo to that position.
*/
class Servo {
    /** Driver. */
    Adafruit_PWMServoDriver driver;
    /** Slot on PWM. */
    uint8_t number;
    /** Maximal possible position of servo arm. */
    const uint16_t servoMaxPosition = SERVOMAX;
    /** Minimal possible position of servo arm. */
    const uint16_t servoMinPosition = SERVOMIN;
    /** Default start position of servo arm. */
    uint16_t servoPosition = (servoMinPosition + servoMaxPosition) / 2;
  public:
    /**
       Constructor.
       @param aDrriver Adafruit_PWMServoDriver - Servo driver.
       @param aNumber uint8_t - Slot on PWM.
    */
    Servo(const Adafruit_PWMServoDriver aDriver, const uint8_t aNumber) {
      driver = aDriver;
      number = aNumber;
    }

    /** Delivers current position. */
    uint16_t getPosition() {
      return servoPosition;
    }

    /** Sets new position. */
    void setPosition(const uint16_t aPosition) {
      if (aPosition < servoMinPosition) {
        servoPosition = servoMinPosition;
      } else if (aPosition > servoMaxPosition) {
        servoPosition = servoMaxPosition;
      } else {
        servoPosition = aPosition;
        driver.setPWM(number, 0, servoPosition);
      }
    }

    /** Moves servo a given step. */
    void move(const int16_t aStep) {
      uint16_t tPosition = servoPosition;
      tPosition += aStep;
      setPosition(tPosition);
    }
};

/** Servo driver. */
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

/** Joystick borders. Joystick positions between border 3 and 4 do not result in servo motion.
  The farer the joystick veers away from its middle the faster the servos move. */
uint16_t yAxisBorders[8] { 895, 770, 635, 542, 482, 355, 250, 125 };
uint16_t xAxisBorders[8] { 895, 767, 659, 531, 470, 348, 245, 120 };

/* Servos. */
Servo servo0(servoDriver, 15);
Servo servo1(servoDriver, 14);
Servo servo2(servoDriver, 13);
Servo servo3(servoDriver, 12);

/* Mean servo position. */
uint16_t servoMeanPosition = (SERVOMAX + SERVOMIN) / 2;

void setup() {
  Serial.begin(9600);
  /*
  pinMode(SW_PIN_0, INPUT);
  digitalWrite(SW_PIN_0, HIGH);
  */

  /* Init servo driver. */
  servoDriver.begin();
  servoDriver.setPWMFreq(60);
  /* Init servos with start positions. */
  servo0.setPosition(servoMeanPosition - 70);
  servo1.setPosition(servoMeanPosition + 200);
  servo2.setPosition(servoMeanPosition);
  servo3.setPosition(servoMeanPosition - 180);
  delay(100);
}

void loop() {
  /* Receive joystick commands. */
  const int8_t rotate = moveCommand(yAxisBorders, Y_PIN_0);
  const int8_t upperArm = moveCommand(xAxisBorders, X_PIN_0);
  const int8_t foreArm = moveCommand(xAxisBorders, X_PIN_1);
  const int8_t hand = moveCommand(yAxisBorders, Y_PIN_1);

  /* Move servos if command received. */
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
    /* Slowest server dictates the speed. */
    uint8_t absUpperArm = abs(upperArm);
    uint8_t absRotate = abs(rotate);
    uint8_t absForeArm = abs(foreArm);
    uint8_t absHand = abs(hand);
    delay(max(absUpperArm, max(absRotate, max(absForeArm, absHand))));
  } else {
    delay(100);
  }
  /*
    Serial.print("rotate: ");
    Serial.println(servo0.getPosition());
    Serial.print("upperArm: ");
    Serial.println(servo1.getPosition());
    Serial.print("foreArm: ");
    Serial.println(servo2.getPosition());
    Serial.print("hand: ");
    Serial.println(servo3.getPosition());
  */
}

/*

*/
int8_t moveCommand(const uint16_t axisBorders[8], const uint8_t pin) {
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
