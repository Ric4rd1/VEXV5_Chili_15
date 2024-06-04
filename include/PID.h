
#include <iostream>

class PID
{
  private:
  double stock_kPID[3];
  public:
  PID();
  PID(double kPID[]);
  PID(double kP, double kI, double kD);
  // PID(double kP, double kI, double kD, double dt, double max, double min);
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double dT = .02;
  double max = 12;
  double min = -12;
  double position = 0;
  double target = 0;
  double error = 0;
  double integral = 0;
  double derivative = 0;
  double prevError = 0;
  double prevPosition = 0;
  double prevDeriv = 0;
  double acceleration = 0;
  double Pout = 0;
  double Iout = 0;
  double Dout = 0;
  double output = 0;
  void clear();
  void resetPID();
  void setPID(double kPID[]);
  void setPID(double kP, double kI, double kD);
  void setTarget(double iTarget);
  double calc(double target, double position);
  // friend std::ostream& operator<<(std::ostream& os, const PID& coordinate)
  // {
  //     os 
  //     << "kP: " << coordinate.x() 
  //     << ", kI: " << coordinate.y() 
  //     << ", dD: " << coordinate.z() 
  //     << ", theta_rad: " << coordinate.theta() 
  //     << ", theta_deg: " << coordinate.theta(feature::Coordinate::rotationType::DEG) 
  //     << ", magn: " << hypot(coordinate.x(), coordinate.y());
  //     return os;
  // }
};

PID::PID()
{
  this->kP = stock_kPID[0] = 0;
  this->kI = stock_kPID[1] = 0;
  this->kD = stock_kPID[2] = 0;
};
PID::PID(double kP, double kI, double kD)
{
  this->kP = stock_kPID[0] = kP;
  this->kI = stock_kPID[1] = kI;
  this->kD = stock_kPID[2] = kD;
};
PID::PID(double kPID[])
{
  this->kP = stock_kPID[0] = kPID[0];
  this->kI = stock_kPID[1] = kPID[1];
  this->kD = stock_kPID[2] = kPID[2];
};

void PID::setTarget(double iTarget) { target = iTarget; }

void PID::setPID(double kP, double kI, double kD)
{
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
}

void PID::setPID(double kPID[])
{
  this->kP = kPID[0];
  this->kI = kPID[1];
  this->kD = kPID[2];
};

double PID::calc(double target, double position) 
{
  this->target = target;
  this->position = position;

  error = target - position;
  // Proportional term
  Pout = kP * error;

  // Integral term
  integral += error * dT;
  Iout = kI * integral;

  // Derivative term
  derivative = (error - prevError)/dT;
  // if(std::abs(derivative) > std::abs(acceleration + prevDeriv) && prevDeriv != 0) // Optional to smooth Derivative calculations and prevent derivative kick
  //   derivative = acceleration + prevDeriv; // Acceleration is added to previous derivative to predict the current derivative and then compare it to the calculated derivative
  Dout = kD * derivative;

  // Calculate total output
  output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > max)
    output = max;
  else if (output < min)
    output = min;

  double integralCap = 3.5;

  if (Iout > integralCap)
    Iout = integralCap;
  else if (Iout < -integralCap)
    Iout = -integralCap;
  // if (std::abs(error) < tolerance) // Optional, if your PID doesn't stop and blows past the target, this prevents Integral windup, 
  // {                                // highly recommended for PIDs that don't fight gravity (e.g. drive train PID, PID on your base).
  //   integral = 0;                  // However, this should most likely be implemented in the movement control function rather than in this general use class
  //   output = 0;                    // since it can interefere with other PID objects where you do want the PID windup 
  // }                                // like in situations where a motor is fighting gravity, (e.g. Lift)

  // Save error to previous error
  acceleration = derivative - prevDeriv;
  prevDeriv = derivative;
  prevPosition = position;
  prevError = error;

  return output;
}

void PID::resetPID()
{
  kP = stock_kPID[0];
  kI = stock_kPID[1];
  kD = stock_kPID[2];
}

void PID::clear()
{
  dT = .02;
  max = 12;
  min = -12;
  position = 0;
  target = 0;
  error = 0;
  integral = 0;
  derivative = 0;
  prevError = 0;
  prevPosition = 0;
  prevDeriv = 0;
  acceleration = 0;
  Pout = 0;
  Iout = 0;
  Dout = 0;
  output = 0;
}
