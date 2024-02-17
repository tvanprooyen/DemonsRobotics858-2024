package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;

public class ControlIntake extends Command {

    private IntakeSubsystem INTAKESUBSYSTEM;
    private IntakePos intakePos;

    public ControlIntake(IntakeSubsystem INTAKESUBSYSTEM, IntakePos intakePos) {
        this.INTAKESUBSYSTEM = INTAKESUBSYSTEM;
        this.intakePos = intakePos;
    }

    @Override
    public void execute() {
        INTAKESUBSYSTEM.setIntakePose(intakePos);
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
