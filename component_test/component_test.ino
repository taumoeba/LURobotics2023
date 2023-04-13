#define DC_DIR1 2
#define DC_DIR2 4
#define DC_DIR3 6
#define DC_DIR4 30
#define DC_DIR5 31 // door
#define DC_PWM1 3
#define DC_PWM2 5
#define DC_PWM3 7
#define DC_PWM4 8
#define DC_PWM5 9 // door

/*
#define STEP_DIR1 23
#define STEP1 22
#define MS1_1 21
#define MS2_1 20
#define EN1 17
#define SLP1 16
#define STEP_DIR2 15
#define STEP2 14
#define MS1_2 41
#define MS2_2 40
#define EN2 39
#define SLP2 38
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(DC_DIR1, OUTPUT);
  pinMode(DC_DIR2, OUTPUT);
  pinMode(DC_DIR3, OUTPUT);
  pinMode(DC_DIR4, OUTPUT);
  pinMode(DC_DIR5, OUTPUT);
  pinMode(DC_PWM1, OUTPUT);
  pinMode(DC_PWM2, OUTPUT);
  pinMode(DC_PWM3, OUTPUT);
  pinMode(DC_PWM4, OUTPUT);
  pinMode(DC_PWM5, OUTPUT);
/*
  pinMode(STEP_DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(SLP1, OUTPUT);
  pinMode(STEP_DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(SLP2, OUTPUT);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(SLP1, HIGH);
  digitalWrite(SLP2, HIGH);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  */
}

void loop() {
    
  // DC MOTORS
  digitalWrite(DC_DIR1, HIGH);
  digitalWrite(DC_DIR2, HIGH);
  digitalWrite(DC_DIR3, HIGH);
  digitalWrite(DC_DIR4, LOW);
  analogWrite(DC_PWM1, 127);
  analogWrite(DC_PWM3, 127);
  delay(1000);
  analogWrite(DC_PWM1, 0);
  analogWrite(DC_PWM3, 0);
  delay(2000);
  analogWrite(DC_PWM2, 127);
  analogWrite(DC_PWM4, 127);
  delay(1000);
  analogWrite(DC_PWM2, 0);
  analogWrite(DC_PWM4, 0);
  delay(2000);
    digitalWrite(DC_DIR1, LOW);
  digitalWrite(DC_DIR2, LOW);
  digitalWrite(DC_DIR3, LOW);
  digitalWrite(DC_DIR4, HIGH);
  analogWrite(DC_PWM1, 127);
  analogWrite(DC_PWM3, 127);
  delay(1000);
  analogWrite(DC_PWM1, 0);
  analogWrite(DC_PWM3, 0);
  delay(2000);
  analogWrite(DC_PWM2, 127);
  analogWrite(DC_PWM4, 127);
  delay(1000);
  analogWrite(DC_PWM2, 0);
  analogWrite(DC_PWM4, 0);
  delay(2000);
  
/*
    // STEPPER MOTORS
    digitalWrite(STEP_DIR1, LOW);
    digitalWrite(STEP_DIR2, LOW);
    for(int i=0; i<200; i++)
    {
        digitalWrite(STEP1, HIGH);
        digitalWrite(STEP2, HIGH);
        delay(0.1);
        digitalWrite(STEP1, LOW);
        digitalWrite(STEP2, LOW);
        delay(0.1);
    }
    delay(2000);
    digitalWrite(STEP_DIR1, HIGH);
    digitalWrite(STEP_DIR2, HIGH);
    for(int i=0; i<200; i++)
    {
        digitalWrite(STEP1, HIGH);
        digitalWrite(STEP2, HIGH);
        delay(0.1);
        digitalWrite(STEP1, LOW);
        digitalWrite(STEP2, LOW);
        delay(0.1);
    }
    delay(2000);
    */
}
