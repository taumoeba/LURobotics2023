/* Motor control library for 2023 Lipscomb Robotics
*/


#ifndef ARDUINO_H
#define ARDUINO_H

#ifndef SERVO_H
#define SERVO_H

class driveMotors
{
    public:
    driveMotors(int PWM1, int DIR1, int PWM2, int DIR2, int PWM3, int DIR3, int PWM4, int DIR4)
    {
      pwm1 = PWM1;
      pwm2 = PWM2;
      pwm3 = PWM3;
      pwm4 = PWM4;
      dir1 = DIR1;
      dir2 = DIR2;
      dir3 = DIR3;
      dir4 = DIR4;

      pinMode(pwm1, OUTPUT);
      pinMode(pwm2, OUTPUT);
      pinMode(pwm3, OUTPUT);
      pinMode(pwm4, OUTPUT);
      pinMode(dir1, OUTPUT);
      pinMode(dir2, OUTPUT);
      pinMode(dir3, OUTPUT);
      pinMode(dir4, OUTPUT);

      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      digitalWrite(dir3, HIGH);
      digitalWrite(dir4, LOW);
    }
    // speed is 0-255
    void forward(int speed)
    {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir3, HIGH);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm1, speed);
        analogWrite(pwm3, speed);
    }
    // Speed is 0-255
    void backward(int speed)
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir3, LOW);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm1, speed);
        analogWrite(pwm3, speed);
    }
    // speed is 0-255
    void right(int speed)
    {
        digitalWrite(dir2, HIGH);
        digitalWrite(dir4, HIGH);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm2, speed);
        analogWrite(pwm4, speed);
    }
    // speed is 0-255
    void left(int speed)
    {
        digitalWrite(dir2, LOW);
        digitalWrite(dir4, LOW);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm2, speed);
        analogWrite(pwm4, speed);
    }
    // dir=0 is left, dir=1 is right. speed is 0-255
    void turn(int dir, int speed) // 0 is left, 1 is right (test)
    {
      // experiment with directions but i think they should all be the same
      digitalWrite(dir1, dir);
      digitalWrite(dir2, dir);
      digitalWrite(dir3, dir);
      digitalWrite(dir4, dir);
      analogWrite(pwm1, speed);
      analogWrite(pwm2, speed);
      analogWrite(pwm3, speed);
      analogWrite(pwm4, speed);
    }

    void stop()
    {
      analogWrite(pwm1,0);
      analogWrite(pwm2,0);
      analogWrite(pwm3,0);
      analogWrite(pwm4,0);
    }

    private:
    int pwm1, dir1, pwm2, dir2, pwm3, dir3, pwm4, dir4;
};

class Arm
{
  public:
  Arm(int pwm, int dir, int step, int ms1, int ms2, int en, int slp)
  {
    pinMode(_dir, OUTPUT);
    pinMode(_step, OUTPUT);
    pinMode(_ms1, OUTPUT);
    pinMode(_ms2, OUTPUT);
    pinMode(_en, OUTPUT);
    pinMode(_slp, OUTPUT);
    pinMode(pwm, OUTPUT);

    raiseServo.attach(pwm);
    _dir = dir;
    _step = step;
    _ms1 = ms1;
    _ms2 = ms2;
    _en = en;
    _slp = slp;

    // en disables outputs when high
    // slp minimizes power consumption when low
    // rst resets internal translator when low
    digitalWrite(_dir, LOW);
    digitalWrite(_en, LOW);
    digitalWrite(_slp, LOW);
    digitalWrite(_step, LOW);
    // starts up in sleep mode
  }

  void raise(int degrees)
  {
    for(int i=0; i<degrees; i++)
    {
      raiseServo.write(degrees);
      delay(0.1);
    }
  }

  // negative degrees to go counterclockwise
  void rotate(int degrees)
  {
    // optional microstepping
    // digitalWrite(MS1, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
    // digitalWrite(MS2, HIGH);
    if(degrees<0) digitalWrite(_dir, HIGH);
    else digitalWrite(_dir, LOW);
    int steps = (abs(degrees)/360)*200;
    for(int i=0; i<steps; i++)
    {
      digitalWrite(_step, HIGH);
      delay(0.1);
      digitalWrite(_step, LOW);
      delay(0.1);
    }
  }

  void sleep()
  {
    digitalWrite(_slp, LOW);
  }

  void wake()
  {
    digitalWrite(_slp, HIGH);
  }

  private:
  Servo raiseServo;
  int _dir, _step, _ms1, _ms2, _en, _slp;
};

class Claw
{
  public:
  Claw(int PWM)
  {
    pinMode(PWM, OUTPUT);
    myservo.attach(PWM);
  }

  void openClaw()
  {
    for(int i=0; i<openDegrees; i++)
    {
      myServo.write(i);
      delay(0.1);
    }
  }

  void closeClaw()
  {
    for(int i=openDegrees; i>closedDegrees; i--)
    {
      raiseServo.write(i);
      delay(0.1);
    }
  }

  private:
  Servo myservo;
  const int closedDegrees = 0; // subject to change
  const int openDegrees = 180; // subject to change
};

class Turntable
{
  public:
  Turntable(int doorDir, int doorPWM, int dir, int step, int ms1, int ms2, int en, int slp)
  {
    _doorDir = doorDir;
    _doorPWM = doorPWM;
    _dir = dir;
    _step = step;
    _ms1 = ms1;
    _ms2 = ms2;
    _en = en;
    _slp = slp;

    pinMode(_doorDir, OUTPUT);
    pinMode(_doorPWM, OUTPUT);
    pinMode(_dir, OUTPUT);
    pinMode(_step, OUTPUT);
    pinMode(_ms1, OUTPUT);
    pinMode(_ms2, OUTPUT);
    pinMode(_en, OUTPUT);
    pinMode(_slp, OUTPUT);

    digitalWrite(_doorDir, HIGH);

    // en disables outputs when high
    // slp minimizes power consumption when low
    // rst resets internal translator when low
    digitalWrite(_dir, HIGH);
    digitalWrite(_en, LOW);
    digitalWrite(_slp, LOW);
    // starts up in sleep mode
  }

  // Stacks are 0, 1, and 2. 0.5, 1.5, and 2.5 are halfway between each stack.
  void goToStackPos(float dest)
  {
    if(currentStack-dest>0) 
    {
      digitalWrite(_dir, HIGH); // go clockwise
      while(currentStack!=dest)
      {
        for(int i=0; i<stepChunk; i++)
        {
          digitalWrite(_step, HIGH);
          delay(0.1);
          digitalWrite(_step, LOW);
          delay(0.1);
        }
        currentStack -= 0.5;
      }
    }
    else 
    {
      digitalWrite(_dir, LOW); // go counter-clockwise
      while(currentStack!=dest)
      {
        for(int i=0; i<stepChunk; i++)
        {
          digitalWrite(_step, HIGH);
          delay(0.1);
          digitalWrite(_step, LOW);
          delay(0.1);
        }
        currentStack += 0.5;
      }
    }
  }

  void openDoor()
  {
    digitalWrite(_doorDir, HIGH); // test direction
    analogWrite(_doorPWM, 127); // test speed
    delay(1); // test duration
    analogWrite(_doorPWM, 0);
  }

  void closeDoor()
  {
    digitalWrite(_doorDir, LOW); // test direction
    analogWrite(_doorPWM, 127); // test speed
    delay(1); // test duration
    analogWrite(_doorPWM, 0);
  }

  void sleep()
  {
    digitalWrite(_slp, LOW);
  }

  void wake()
  {
    digitalWrite(_slp, HIGH);
  }

  private:
  int _doorDir, _doorPWM, _dir, _step, _ms1, _ms2, _en, _slp;
  const int stepChunk = (60/360)*200;
  // stack over hole
  float currentStack = 0.0;
};

class duckStorage
{
  public:
  duckStorage(int solenoid, int pwm1, int pwm2)
  {
    _solenoid = solenoid;
    _pwm1 = pwm1;
    _pwm2 = pwm2;

    pinMode(_solenoid, OUTPUT);
    pinMode(_pwm1, OUTPUT);
    pinMode(_pwm2, OUTPUT);

    servo1.attach(pwm1);
    servo2.attach(pwm2);
    servo1.write(0);
    servo2.write(0);
  }

  void tilt()
  {
    for(int i=1; i<=tiltDegrees; i++)
    {
      servo1.write(i);
      servo2.write(i);
      delay(100); // adjust as needed
    }
  }

  void release()
  {
    digitalWrite(_solenoid, HIGH);
    // we could probably just get rid of these two lines if we're short on time
    delay(2000);
    digitalWrite(_solenoid, LOW);
  }

  private:
  Servo servo1;
  Servo servo2;
  int _solenoid, _pwm1, _pwm2;
  const int tiltDegrees = 30;
};

#endif