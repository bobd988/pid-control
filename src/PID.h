#ifndef PID_H
#define PID_H

class PID {
private:
  // Error
  double _p_error;
  double _i_error;
  double _d_error;
  double _prev_cte;

  long _count;
  double _error_sum;

  double _Kp;
  double _Ki;
  double _Kd;

public:

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  *  Returns the average error.
  */
  double AverageError();

};

#endif /* PID_H */
