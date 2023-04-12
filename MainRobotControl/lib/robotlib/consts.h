// Constants used in the main robot algorithm
const int driveSpeed = 127; // test
enum ControlState {foodChipDropoff, searching, intercepting, grabbing, recycling};
//const int driveChunkTime = 2000; // milliseconds, change
const int clawCenterWindow = 50;
const int armOffsetPixels = 50; // 75mm at ideal distance in pixels, TEST
const int milliPerMilli = 10; // millisecond delay per millimeter travel. TEST AND UPDATE
const bool DEBUG = true;
const int idealClawGrabDistance = 130; // millimeters from robot body
const int pedestalWidthAtIdealDistance = 50; // pixy pixels, TEST
const int duckWidthAtIdealDistance = 100; // pixy pixels, TEST