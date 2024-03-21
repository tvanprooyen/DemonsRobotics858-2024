package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class TimeOutTimer extends Timer {

    private double setTime;

    public TimeOutTimer() {
        super();
        this.setTime = 0;
    }

    /**
     * Set the time to be added to the current time
     * @param setTime in seconds
     */
    public void setTime(double setTime) {
        this.setTime = setTime;
    }

    @Override
    public double get() {
        return super.get() + this.setTime;
    }
    
    public void zero() {
        this.setTime = 0;
        reset();
    }
}
