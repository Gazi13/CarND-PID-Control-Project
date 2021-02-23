#include "PID.h"
#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <vector>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    
    p[0] = Kp_;
    p[1] = Ki_;
    p[2] = Kd_;

    //paramets for twiddle use
    best_err = 0;
    total_error = 0;

    // values to start 'twiddling' with
    dp[0] = 0.025; // p
    dp[1] = 0.0005; // i
    dp[2] = 0.25; // d

    flag = 0;
    pid_index = 0;
    cout<<"Intialized!!"<<endl;


}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    //return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
    return (-p[0] * p_error) + (-p[1] * i_error) + (-p[2] * d_error);
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


void PID::Twiddle(double total_error) {

    d_error = 0;
    i_error = 0;

    //this if condition is only run once in the beginning
    if (flag == 0){
        flag = 1;
        best_err = total_error;
        p[pid_index] += dp[pid_index];
    }

    if (flag == 1){
        if (total_error < best_err ){
            best_err = total_error;
            dp[pid_index] *= 1.1;
            // move on to next parameter in PID
            pid_index = (pid_index+1)%3;
            p[pid_index] += dp[pid_index];
        }
        else{
            p[pid_index] -= 2*dp[pid_index];
            flag = 2;
        }
     }

    if (flag == 2){
        if (total_error < best_err ){
            best_err = total_error;
            dp[pid_index] *= 1.1;
            // move on to next parameter in PID
            pid_index = (pid_index+1)%3;
            p[pid_index] += dp[pid_index];
            flag = 1;
        }
        else{
            p[pid_index] += dp[pid_index];
            dp[pid_index] *= 0.9;
            // move on to next parameter in PID
            pid_index = (pid_index+1)%3;
            p[pid_index] += dp[pid_index];
            flag = 1 ;
        }
    }
  
  cout<< "Total_error " << total_error <<" Best Error "<< best_err<<" Kp: "<< p[0]<<" Ki: "<<p[1] <<" Kd: "<<p[2] << " dp " << dp[pid_index] <<endl;
}
