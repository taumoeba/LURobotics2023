#include "robotlib.h"
#include "pins.h"
#include "consts.h"
#include "Adafruit_VL53L0X.h"

// Object declaration
ControlState s = startup;
navigation nav = navigation();
driveMotors drive = driveMotors(DC_PWM1, DC_DIR1, DC_PWM2, DC_DIR2, DC_PWM3, DC_DIR3, DC_PWM4, DC_DIR4);
Arm arm = Arm(PWM6, STEP_DIR1, STEP1, MS1_1, MS2_1, EN1, SLP1);
Claw claw = Claw(PWM7);
Turntable turntable = Turntable(DC_DIR5, DC_PWM5, STEP_DIR2, STEP2, MS1_2, MS2_2, EN2, SLP2);
duckStorage ducks = duckStorage(SOL, PWM8, PWM9);

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

void readDistance()
{
  lox1.rangingTest(&measure1, false); 
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);
}

void setup() {
  // Pin setup
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  pinMode(XSHUT4, OUTPUT);


  // Serial setup
  Serial.begin(115200);
  Serial.println("Starting...");

  // Distance sensor setup
  // reset sensors
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  delay(10);
  digitalWrite(XSHUT1, HIGH);
  digitalWrite(XSHUT2, HIGH);
  digitalWrite(XSHUT3, HIGH);
  digitalWrite(XSHUT4, HIGH);

  // set sensor addresses one-by-one
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  lox1.begin(0x30);
  digitalWrite(XSHUT2, HIGH);
  lox2.begin(0x31);
  digitalWrite(XSHUT3, HIGH);
  lox3.begin(0x32);
  digitalWrite(XSHUT4, HIGH);
  lox4.begin(0x33);

  // initial position reading
  readDistance();
  Serial.print("Sensor 1: ");
  Serial.println(measure1.RangeMillimeter);
  Serial.print("Sensor 2: ");
  Serial.println(measure3.RangeMillimeter);
  Serial.print("Sensor 3: ");
  Serial.println(measure3.RangeMillimeter);
  Serial.print("Sensor 4: ");
  Serial.println(measure4.RangeMillimeter);

  // Activate dummy bot


}

void loop() {
  // execute operations according to control state
  switch(s)
  {
    case foodChipDropoff:
      // Go to square one
      drive.left();
      while(measure4.RangeMillimeter > 100) readDistance();
      drive.stop();
      // Drop first batch of chips
      turntable.goToStackPos(1.0);
      turntable.openDoor();
      turntable.closeDoor();
      // Go to square two
      drive.forward();
      while(measure1.RangeMillimeter > 100) readDistance();
      drive.stop();
      // Drop second batch of chips
      turntable.goToStackPos(2.0);
      turntable.openDoor();
      turntable.closeDoor();
      // Switch to next state
      s = searching;
      break;
    case searching:
      break;
    case intercepting:
      break;
    case grabbing:
      break;
    case sorting:
      break;
    case dropping:
      break;
    case recycling:
      break;
  }
  
}
