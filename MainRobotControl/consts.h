// Constants used in the main robot algorithm
const int driveSpeed = 127; // test
enum ControlState {foodChipDropoff, searching, intercepting, grabbing, sorting, dropping, recycling};
const int driveChunkTime = 2000; // milliseconds, change