#ifndef PID_H
#define PID_H
#include <vector>
#include <queue>
#include <string>
using namespace std;

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
    string pid_type;
    /*
    * Errors vector
    */
    std::vector<double>param;
    /*
    * Coefficients vector
    */
    std::vector<double> dp;

    //std::queue<double> steers;
    queue<double> steers;

    int i;
    int k;

    double last_cte;
    double int_cte;
    double best_err;
    /*
    * Constructor
    */        
    PID(double Kp=0, double Ki=0, double Kd=0, string type="");

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, string type);

    /*
    * Update the PID error variables given cross track error.
    */
    double UpdateError(double cte);


    void twiddle(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
};

#endif /* PID_H */
