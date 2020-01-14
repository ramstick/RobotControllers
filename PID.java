public class PID{
    
    double Kp, Kd, Ki;
    double intE, dE;

    double end;
    double lastT, lastE;

    double tolerance;
    /**
     * 
     * @param Kp - The proportional parameter
     * @param Kd - The derivative parameter
     * @param Ki - The integral parameter
     * @param end - The desired state
     * @param tolerance - How much it error it will tolerate and call it done.
     */
    public PID(double Kp, double Kd, double Ki, double end, double tolerance){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.end = end;
        lastE = 0;
        this.dE = 0;
        intE = 0;
        this.tolerance = tolerance;
    }
    /**
     * 
     * @param current - The current position of the PID controlled
     * @param timeT - The current time This is cool
     * @return   - The speed the motor should be set at
     */
    public double output(double current, double timeT){
        
        if(checkDone(current)){
            return 0;
        }
        double error = end - current;
        double dt = timeT - lastT;
        intE += error * dt;
        lastE = error;
        lastT = timeT;
        return Kp * error + Ki * intE + Kd * (error-lastE) / dt;
    }
    /**
     * 
     * @param current - The current position
     * @return - If it has reached its desired end state
     */
    public boolean checkDone(double current){
        return Math.abs(end-current) < tolerance;
    }

}