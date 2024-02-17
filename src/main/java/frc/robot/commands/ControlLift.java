package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftLaunchSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem.ShooterPos;

public class ControlLift extends Command {

    private LiftLaunchSubsystem LLSS;
    private ShooterPos shooterPos;

    public ControlLift(LiftLaunchSubsystem LLSS, ShooterPos shooterPos) {
        this.LLSS = LLSS;
        this.shooterPos = shooterPos;
    }

    @Override
    public void execute() {
        LLSS.setShooterPose(shooterPos);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}