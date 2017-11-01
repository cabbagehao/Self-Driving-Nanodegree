#include "PID.h"

using namespace std;
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>


PID::PID(double Kp, double Ki, double Kd, string type) {
    Init(Kp, Ki, Kd, type);
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, string type) {

    param.push_back(Kp);
    param.push_back(Ki);
    param.push_back(Kd);
    pid_type = type;

    last_cte = 0;
    int_cte = 0;
    // position in p
    i = 0;
    // 0-8, 3 step of p[i].
    k = 0;
}

double PID::UpdateError(double cte) {
    
    if(pid_type=="steer" && int_cte * cte < 0)
        int_cte /= -10;

    if(last_cte ==0) last_cte = cte;
    double diff_cte = cte - last_cte;
    last_cte = cte;
    int_cte += cte;

    double output = param[0]*cte + param[1]*int_cte - param[2]*diff_cte;



    // Get the mean steer_value from last 5 times.
    // if(steers.size() > 5) steers.pop();
    // steers.push(steer_value);
    // queue<double> steers_copy = steers;
    // double steer_sum = 0;
    // for(int i=0; i<steers.size(); i++)
    // {
    //     steer_sum += steers_copy.front();
    //     steers_copy.pop();
    // }
    // steer_value = steer_sum / steers.size();

    // cout << "steer_value: " <<steer_value << endl;

    // Fault tolerance
    if(output > 1) output = 1;
    if(output < -1) output = -1;
    if(pid_type == "steer")
    {
        std::cout << "CTE: " << cte << " output: " << output << " I: " << param[1]*int_cte << " D: " << param[2]*diff_cte << std::endl;   
    }    
    return output;    
}

void PID::twiddle(double cte)
{    

    i = k / 3;
    double new_err;
    double sum = accumulate(dp.begin() , dp.end(), 0);
    if(abs(sum) < 0.2 && abs(cte) < 1)
    {
        cout << "sum dp < 0.2, cte < 1" << endl;
        return;
    }
    // step1
    if(k == 0 || k == 3 || k == 6 ) best_err = cte*cte;
    
    // step2
    if(k == 1 || k == 4 || k == 7 )
    {
        param[i] += dp[i];
        new_err = cte*cte;
        // better, modify dp[i], goto i+1
        if(new_err <= best_err)
        {
            dp[i] *= 1.1;

            k = (k+2) % 9;
            return;
        }
        else
        {
            param[i] -= 2*dp[i];

        }  
    }
    // step3
    if(k == 2 || k == 5 || k == 8 )
    {
        // better, modify dp[i], goto i+1
        new_err = cte*cte;
        if(new_err < best_err)
        {
            dp[i] *= 1.1;
        }
        else
        {
            param[i] += dp[i];
            dp[i] *= 0.9;
        }
    }
     
    k = (k+1) % 9;
}


