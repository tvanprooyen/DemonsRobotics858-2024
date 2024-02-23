package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class TimeOutTimer extends Timer {

    private double addTime;

    public TimeOutTimer() {
        super();
        this.addTime = 0;
    }

    public void addTime(double addTime) {
        this.addTime = addTime;
    }

    @Override
    public double get() {
        return super.get() + this.addTime;
    }
}
