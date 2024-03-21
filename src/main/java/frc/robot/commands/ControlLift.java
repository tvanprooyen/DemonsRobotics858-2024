package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftLaunchSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem.ShooterPos;
import frc.robot.subsystems.IntakeSubsystem;

public class ControlLift extends Command {

    private LiftLaunchSubsystem LLSS;
    private IntakeSubsystem ISS;
    private ShooterPos shooterPos;

    public ControlLift(LiftLaunchSubsystem LLSS, ShooterPos shooterPos) {
        this.LLSS = LLSS;
        this.shooterPos = shooterPos;
    }

    @Override
    public void execute() {
         /*if(ISS.runIntakeAuto()){ //runs the intake auto untill in finishes then it points up and shoots
             LLSS.setShooterPose(shooterPos);
         }*/
        LLSS.setShooterPose(shooterPos);
    }

    @Override
    public void end(boolean interrupted) {
        LLSS.setShooterPose(ShooterPos.Store);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}