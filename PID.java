public class PID{

    double Kp, Kd, Ki;
    double intE, dE;

    double end;
    double lastT, lastE;

    boolean done;

    double tolerance;

    public PID(double Kp, double Kd, double Ki, double end, double tolerance){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.end = end;
        lastError = 0;
        this.dE = 0;
        intE = 0;
        done = false;
        this.tolerance = tolerance;
    }

    public double output(double current, double timeT){
        checkDone(current);
        if(done){
            return 0;
        }
        double error = end - current;
        double dt = timeT - lastT;
        intE += error * dt;
        lastE = error;
        lastT = timeT;
        return Kp * error + Ki * intE + Kd * (error-lastE) / dt;
    }
    public void checkDone(double current){
        done = Math.abs(end-current) < tolerance;
    }

}