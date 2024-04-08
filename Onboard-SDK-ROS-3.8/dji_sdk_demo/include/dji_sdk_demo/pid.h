#ifndef _MYPID_H_
#define _MYPID_H_
// #include <iostream>
#include <cmath>
class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        void reset_control();
        void useint();
        void noint();
        
    private:
        double _max;
        double _min;
        double _dt;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _intLimit;
        bool use_int;
};
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID(){};
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );
        void useint();
        void noint();
        // Returns the manipulated variable given a setpoint and current process value
        void reset_control();
        double calculate( double setpoint, double pv);
        ~PID();

    private:
        PIDImpl *pimpl;
};
PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}

void PID::reset_control()
{
    pimpl->reset_control();
}

void PID::useint(){
    pimpl->useint();
}
void PID::noint(){
    pimpl->noint();
}
PID::~PID() 
{
    delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _intLimit(0.7),
    use_int(true)
{
}

void PIDImpl::reset_control()
{
    _integral = 0;
    _pre_error = 0;
    // use_int=false;
}

void PIDImpl::useint()
{
    use_int=true;
}
void PIDImpl::noint()
{
    use_int=false;
}
double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    double kif = 1;
    if( fabs(error) > 0.5 ) kif = 0.4;

    // Integral term
    
    double Iout=0;
    _integral += error * _dt;
    if(use_int){
        Iout = _Ki * _integral * kif;
    }

    if( Iout > _intLimit ){
        _integral = _intLimit/(_Ki*kif);
        // std::cout << "LIMIT+" << std::endl;
    }
    else if( Iout < -_intLimit ) 
    {
        _integral = -_intLimit/(_Ki*kif);
        // std::cout << "LIMIT+" << std::endl;
    }
    //如果不用积分，去掉积分项
    if(!use_int){
        _integral=0;
    }

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    double output=0;
    // Calculate total output
    if(use_int){
            output = Pout + Iout + Dout;
    }else{
        output = Pout + Dout;
    }
    

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}
#endif
