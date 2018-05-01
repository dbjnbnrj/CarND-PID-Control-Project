#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double square_error;
  int n;
  int num_steps;
  bool is_twiddle, step1, step2;

  double best_square_error;
  double dp[3];

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
  * Update the Twiddle params.
  */
  void UpdateTwiddleParams(double cte);

  void PrintImprovement();

  void UpdateParams(int i, int value);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
