#include "PID.h"
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	best_err = std::numeric_limits<double>::max();

	// start twiddling with Kp (state 0) first
	twiddle_state = 0;
	twiddle_update = 0;

	this->p = {Kp, Ki, Kd};
  this->dp = {0.2, 0.0, 0.2};

	this->Kp = Kp;
	this->Ki = Ki;
  this->Kd = Kd;

  // where steering = -Kp * p_error - Kd * d_error - Ki * i_error
  p_error = 0.0; // proportional cte
  i_error = 0.0; // integral cte
  d_error = 0.0; // differential cte
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	
  
}

double PID::TotalError() {
	return -p[0] * p_error - p[1] * i_error - p[2] * d_error;
}

void PID::twiddle(double total_err) {

	// initialize best_err to collected total_err from track data
	if(best_err == std::numeric_limits<double>::max()){
		//cout << total_err << endl;
		best_err = total_err;
		return;
	}

	if(twiddle_update == 0){
		p[twiddle_state] += dp[twiddle_state];
		twiddle_update = 1;
		return;
	}
	else if ((twiddle_update == 1 || twiddle_update == 2) && total_err < best_err){
		
		// start twiddling
		best_err = total_err;
    dp[twiddle_state] *= 1.1;

    // change twiddle state
    if(twiddle_state == 0){
    	twiddle_state = 1;
    	twiddle_update = 0;
    }
    else if(twiddle_state == 1){
    	twiddle_state = 2;
    	twiddle_update = 0;
    }
    else if(twiddle_state == 2){
    	twiddle_state = 0;
    	twiddle_update = 0;
    }
  }
  else if (twiddle_update == 1 && total_err > best_err){

  	p[twiddle_state] -= 2 * dp[twiddle_state];
  	twiddle_update = 2;
        
  }
  else if (twiddle_update == 2 && total_err > best_err){
  	p[twiddle_state] += dp[twiddle_state];
    dp[twiddle_state] *= 0.9;

    // change twiddle state
    if(twiddle_state == 0){
    	twiddle_state = 1;
    	twiddle_update = 0;
    }
    else if(twiddle_state == 1){
    	twiddle_state = 2;
    	twiddle_update = 0;
    }
    else if(twiddle_state == 2){
    	twiddle_state = 0;
    	twiddle_update = 0;
    }
  }
}

