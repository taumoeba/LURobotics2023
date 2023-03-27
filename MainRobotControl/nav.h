/* Navigation library for 2023 Lipscomb Robotics
*/

//#ifndef ARDUINO_H
//#define ARDUINO_H

class navigation
{
  public:
  navigation()
  {
    pos = [0,0];
  }

  void updatePos(int x, int y)
  {
    pos[1] = x;
    pos[2] = y;
  }

  void moveToWaypoint(int w)
  {
    // do stuff
  }

  private:
  int pos[2];
  int targetPos[2];
  int recyclingPos[2];
  const int waypoints[5][2] = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}}; // update
  int lastWaypoint = 0;
};

//#endif