#ifndef ARDUINO_H
#define ARDUINO_H

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

    void forward(int speed)
    {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir3, HIGH);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm1, speed);
        analogWrite(pwm3, speed);
    }

    void backward(int speed)
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir3, LOW);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm1, speed);
        analogWrite(pwm3, speed);
    }

    void right(int speed)
    {
        digitalWrite(dir2, HIGH);
        digitalWrite(dir4, HIGH);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm2, speed);
        analogWrite(pwm4, speed);
    }

    void left(int speed)
    {
        digitalWrite(dir2, LOW);
        digitalWrite(dir4, LOW);
        
        if(speed > 255) speed = 255;
        if(speed < 0) speed = 0;

        analogWrite(pwm2, speed);
        analogWrite(pwm4, speed);
    }

    private:
    int pwm1, dir1, pwm2, dir2, pwm3, dir3, pwm4, dir4;
};

class 

#endif