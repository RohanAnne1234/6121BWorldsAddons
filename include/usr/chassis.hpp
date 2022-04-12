#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

void setChassisMode(int mode);

void _leftReset();
void _rightReset();

void reset();

void chassisTask(void *);

void setBrakeMode(int mode);

void setChassisMax(int power);

void setTurnMax(int power);

void setAccelStep(int st);

void delayDist(double dist);

void opticalSensorInit();
void updateCurrentBrightness();
bool isBelowThreshold();

void leftWaitUntilSettled();
void rightWaitUntilSettled();

void turnWaitUntilSettled();

void chassisWaitUntilSettled();

void moveForward(double distance_inches);
void moveForwardAsync(double distance_inches);

void moveBack(double distance_inches);
void moveBackAsync(double distance_inches);

void turn(double degrees);
void turnAsync(double degrees);

void leftMove(double iTarget);
void leftMoveAsync(double iTarget);
void rightMove(double iTarget);
void rightMoveAsync(double iTarget);

void pointTurnAsync(bool isRight, double angle);

void pointTurn(bool isRight, double angle);

void arcTurnAsync(double rad, double angle);
void arcTurn(double rad, double angle);




#endif
