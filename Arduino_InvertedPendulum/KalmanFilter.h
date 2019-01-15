// Cf.: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

class KalmanFilter
{
private:
  double angle;
  double bias;
  double P[2][2];

public:
  KalmanFilter();
  void setAngle(double newAngle);
  double calcAngle(double newAngle, double newRate, double dt);
};