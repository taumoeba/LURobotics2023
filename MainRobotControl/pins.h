// Pin definitions

// DC Motors
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

// SPI
#define CS0 10
#define SCK0 13
#define MOSI0 11
#define MISO0 12
#define CS1 0
#define SCK1 27
#define MOSI1 26
#define MISO1 1

// Stepper Motors
// stepper 1 is arm rotate, stepper 2 is turntable
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

// Servo Motors
#define PWM6 24 // arm raising servo
#define PWM7 25 // claw servo
#define PWM8 28 // duck servo 1
#define PWM9 29 // duck servo 2

// I2C
#define SCL 19
#define SDA 18

// Distance sensors
#define XSHUT1 37
#define XSHUT2 36
#define XSHUT3 35
#define XSHUT4 34

// photodiode
#define D_PHOTO 33

// solenoid
#define SOL 32