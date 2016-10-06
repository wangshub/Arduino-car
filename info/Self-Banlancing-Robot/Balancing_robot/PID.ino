
// PID variables
int outMax = 255;
int outMin = -255;
float lastInput = 0;
double ITerm =0;

// PID constants
// You can change this values to adjust the control
double kp = 100;         // Proportional value
double ki = 2;           // Integral value
double kd = 0;           // Derivative value

double Setpoint = 0;     // Initial setpoint is 0

// Calculates the PID output
double Compute(double input)
{

      double error = Setpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      // Compute PID Output
      double output = kp * error + ITerm + kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  
      // Remember some variables for next time
      lastInput = input;
      return output;
}


// Seters and geters for Setpoint
void SetSetpoint(double d){
  
  Setpoint = d; 
  
}

double GetSetPoint(){
  
  return Setpoint;

}
