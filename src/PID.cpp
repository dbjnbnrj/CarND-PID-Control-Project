#include "PID.h"
#include <iostream>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
  PID::Kp = _Kp;
  PID::Ki = _Ki;
  PID::Kd = _Kd;
  PID::p_error = 0.0;
  PID::prev_cte = -1.0;
  PID::i_error = 0.0;
  PID::d_error = 0.0;

  // All new twiddle params
  PID::is_twiddle = false;
  PID::step1 = false;
  PID::step2 = false;
  PID::square_error = 0.0;
  PID::n = 0;
  dp = {0.1 * Kp, 0.1* Ki, 0.1* Kd}
  PID::best_square_error = 1.0;
}

void PID::UpdateError(double cte) {
  if(prev_cte == -1.0){
    d_error = cte - cte;
  } else {
    d_error = cte - prev_cte;
  }
  p_error = cte;
  i_error += cte;
  prev_cte = cte;

  if(is_twiddle){
    square_error = ((cte * cte) + square_error*n) / (n+1) ;
    cout << "Square error is " << square_error << endl;
    //double new_square_error = 0.0;
    if (square_error < best_square_error) {
        best_square_error = square_error;
        dp[n%3] *= 1.1;
        step1 = false;
        step2 = false;
    }
    if (!step1 && !step2) {
      UpdateParams(n%3, dp[n%3]);
      step1 = true;
    } else if (step1 && !step2) {
      UpdateParams(n%3, -2*dp[n%3]);
      step2 = true;
    } else {
      UpdateParams(n%3, dp[n%3]);
      dp[n%3] *= 0.9;
      step1 = step2 = false;
    }
    n += 1;
  }

}

void PID::PrintImprovement(){
  if( TotalError() < best_square_error){
    cout << "------" << endl << " Improved Best Square Error Is " << best_square_error << " And Params Are ("
    << Kp << ", " << Ki << ", " << Kd << ")" << endl << "------" << endl;
  } else {
    cout << "NO IMPROVEMENTS NOTED" << endl;
  }
}

void PID::UpdateParams(int i, int value){
  if (i == 0) {
    cout << "updated KP";
    Kp += value;
  } else if (i == 1) {
    cout << "updated KI";
    Ki += value;
  } else {
    cout << "updated KD";
    Kd += value;
  }
  PrintImprovement();
}


double PID::TotalError() {
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}
