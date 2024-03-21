package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private double IntakeDeploy;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double IntakeDeploy){
         this.intakeSubsystem = intakeSubsystem;
         this.IntakeDeploy = IntakeDeploy;

    }

    @Override
    public void execute(){
        intakeSubsystem.setDeploySet(IntakeDeploy);

    }
    
     @Override
    public boolean isFinished(){
        return intakeSubsystem.isInPose();
    }
}