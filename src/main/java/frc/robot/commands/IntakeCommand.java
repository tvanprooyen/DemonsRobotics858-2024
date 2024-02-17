package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private double DeploySet;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double DeploySet){
         this.intakeSubsystem = intakeSubsystem;
         this.DeploySet = DeploySet;

    }

    @Override
    public void execute(){
        intakeSubsystem.setDeploySet(DeploySet);

    }
    
     @Override
    public boolean isFinished(){
        return intakeSubsystem.isInPose();
    }
}