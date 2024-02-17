package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotor extends Command {
    private double MotorSpeed;
    private IntakeSubsystem INTAKESUBSYSTEM;

    public IntakeMotor(double MotorSpeed, IntakeSubsystem INTAKESUBSYSTEM) {
        this.MotorSpeed = MotorSpeed;
        
        this.INTAKESUBSYSTEM = INTAKESUBSYSTEM;
    }

    @Override
    public void execute() {
        INTAKESUBSYSTEM.setIntakeSpeed(MotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        INTAKESUBSYSTEM.setIntakeSpeed(0);
    }
}
