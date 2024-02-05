package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private double IntakeSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double IntakeSpeed){
        this.intakeSubsystem = intakeSubsystem;
        this.IntakeSpeed = IntakeSpeed;
    }
    
    @Override
    public void execute(){
        intakeSubsystem.RotateIntake(Math.toRadians(90));
        if(!intakeSubsystem.getIRSensor()){
            intakeSubsystem.setIntakeSpeed(IntakeSpeed);
        }

    } 

    @Override
    public void end(boolean interrupted) {
       intakeSubsystem.RotateIntake(Math.toRadians(0));
    //    if(intakeSubsystem.getIRSensor() && intakeSubsystem.getLimitSwitch()){
    //     intakeSubsystem.FeedIntake(0.5);
    //    }
    intakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}

