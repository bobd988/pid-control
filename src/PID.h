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

    double output_value;
    double dp_p;
    double dp_i;
    double dp_d;
    int step_counter;
    int max_steps;
    double total_err;
    double best_err;
    bool increment = true;
    int coefficient_choice;

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
    * Twiddle.
    */
    void Twiddle();
};

#endif /* PID_H */