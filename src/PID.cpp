#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;

    this->output_value = 0;
    this->step_counter = 0;
    this->max_steps = 500;
    this->total_err = 0;
    this->best_err = 1000;

    this->dp_p = 0.9;
    this->dp_i = 0.9;
    this->dp_d = 1;

    this->increasing = true;
    this->cnt = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    output_value = -Kp*p_error - Kd*d_error - Ki*i_error;
    output_value = output_value<-1?-1:output_value;
    output_value = output_value>1?1:output_value;
    // Limit between -1 or 1
    if (output_value > 1.0)
        output_value = 1.0;
    else if (output_value < -1.0)
        output_value = -1.0;

    step_counter++;
    if (step_counter >= max_steps/2) {
        total_err += pow(cte, 2);
    } else {
        total_err += cte;
    }
}

double PID::TotalError() {
    return total_err/step_counter;
}

void PID::Twiddle() {

    if (step_counter == max_steps) {

        double err = TotalError();

        cout << "total_err = " << total_err << endl;
        cout << "best_err = " << best_err << endl;
        cout << "err = " << err << endl;
        cout << "dp_p = " << dp_p << ", dp_i = " << dp_i << ", dp_d = " << dp_d << endl;

        if (err < best_err) {
            int value = cnt %3;
            if (value == 0) {
                if (dp_p < 1)
                    dp_p *= 1.1;
                Ki = Ki * dp_i;
            }
            else if (value ==1){
                if (dp_i < 1)
                    dp_i *= 1.1;
                Kd = Kd * dp_d;
            }
            else if(value ==2){
                if (dp_d < 1)
                    dp_d *= 1.1;
                Kp = Kp * dp_p;
            }
            cnt++;
            best_err = err;
            increasing = true;
        } else {
            cnt++;
            if (increasing) {
                increasing = false;
                int value = cnt %3;
                if(value==0) {
                    Kp = Kp / dp_p * dp_p;
                }else if(value==1) {
                    Ki = Ki / dp_i * dp_i;
                }else if(value==2){
                    Kd = Kd / dp_d*dp_d;
                }
            } else {
                increasing = true;
                int value = cnt %3;
                if(value==0){
                    Kp = Kp * dp_p;
                    Ki = Ki * dp_i;
                }else if(value==1){
                    Ki = Ki * dp_i;
                    Kd = Kd * dp_d;
                }else if (value==2){
                    Kd = Kd *dp_d;
                    Kp = Kp * dp_p;
                }
            }
        }
        step_counter = 0;
    }
    return ;
}

