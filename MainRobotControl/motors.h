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
      raiseServo.attach(pwm);
      _dir = dir;
      _step = step;
      _ms1 = ms1;
      _ms2 = ms2;
      _en = en;
      _slp = slp;

      digitalWrite(_dir, HIGH);
      // do something with enable
      // do something with slp
    }

    void raise(int degrees)
    {
      raiseServo.write(degrees);
    }

    void rotate(int degrees)
    {
      // figure out later
    }

  private:
  Servo raiseServo;
  int _dir;
  int _step;
  int _ms1;
  int _ms2;
  int _en;
  int _slp;
};

class Claw
{
  public:
  Claw(int PWM)
  {
    myservo.attach(PWM);
  }

  void openClaw()
  {
    myservo.write(closedDegrees);
  }

  void closeClaw()
  {
    myservo.write(openDegrees);
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

    digitalWrite(_doorDir, HIGH);

    // set up stepper
  }

  void goToStackPos(float pos)
  {
    // do stuff
  }

  void openDoor()
  {
    // do stuff
  }

  void closeDoor()
  {
    // do stuff
  }

  private:
  int _doorDir, _doorPWM, _dir, _step, _ms1, _ms2, _en, _slp;
  const int nextStackSteps = 60; // just guessing
  float currentStack = 0.0; // 0, 0.5, 1, 1.5, 2, 2.5
};

class duckStorage
{
  public:
  duckStorage(int solenoid, int pwm1, int pwm2)
  {
    _solenoid = solenoid;
    _pwm1 = pwm1;
    _pwm2 = pwm2;

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