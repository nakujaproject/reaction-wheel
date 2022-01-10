class PID{
    public:
        PID(float kp, float ki, float kd){
            _kp = kp;
            _ki = ki;
            _kd = kd;
            previousError = 0;
            error = 0;
            errorSum = 0;
        }
        float getOutput(float currentError){
            previousError = error;
            error = currentError;
            errorSum += error;
            return error * _kp + _kd * (error - previousError) + errorSum * _ki;
        }

    private:
        float _ki;
        float _kp;
        float _kd;
        float error;
        float previousError;
        float errorSum;

};