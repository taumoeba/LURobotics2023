// Constants used in the main robot algorithm
const int driveSpeed = 127; // test
enum ControlState {foodChipDropoff, searching, intercepting, grabbing, recycling};
const int driveChunkTime = 2000; // milliseconds, change
const int clawCenterWindow = 50;
const int armOffset = 100; // distance from center of pixy camera to center of arm, millimeters