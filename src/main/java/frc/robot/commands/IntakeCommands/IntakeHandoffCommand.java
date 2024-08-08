// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LaunchConstants;


// Dictates the Handoff between Intake and Shooter 
public class IntakeHandoffCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private LiftLaunchSubsystem liftLaunchSubsystem;
  private Timer timer;
  private boolean isDone;

  public IntakeHandoffCommand(IntakeSubsystem intakeSubsystem, LiftLaunchSubsystem liftLaunchSubsystem, Timer timer) {
    this.intakeSubsystem = intakeSubsystem;
    this.liftLaunchSubsystem = liftLaunchSubsystem;
    this.timer = timer;


    addRequirements(intakeSubsystem);
    addRequirements(liftLaunchSubsystem);
  }

  @Override
  public void initialize() {

    isDone = false;
    timer.reset();
  }

  @Override
  public void execute() {
    /* // Move intake into position
    if(timer.get() > 0 && timer.get() < 1){ 
      intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_HANDOFF_ANGLE); 

    // Handoff note to feed motors
    } else if(timer.get() > 1 && timer.get() < 1.5){ 
        intakeSubsystem.setIntakeSpeed(0.5);
        liftLaunchSubsystem.setFeedSpeed(0.5);

    // Move intake back ands stop feed motors
    } else if(timer.get() > 1.5 & timer.get() < 3) { 
      intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
      liftLaunchSubsystem.setFeedSpeed(0);
      
    // Put note into position
    } else if(timer.get() > 3  ){ 
       intakeSubsystem.setIntakeSpeed(0);
       isDone = true;
    } */

    if(!intakeSubsystem.getLimitSwitch() && timer.get() == 0) {
      intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_ANGLE);
      intakeSubsystem.setIntakeSpeedTop(0.3);
      intakeSubsystem.setIntakeSpeedBottom(0.3);
      liftLaunchSubsystem.liftSet(LaunchConstants.SHOOTER_HANDOFF_ANGLE);
    } else {
      if (timer.get() >= 0) {
        timer.start();
      }

      if(timer.get() > 0 && timer.get() < 1){ 
        intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_HANDOFF_ANGLE); 

        intakeSubsystem.setIntakeSpeedTop(0);
        intakeSubsystem.setIntakeSpeedBottom(0);

      // Handoff note to feed motors
      } else if(timer.get() > 1 && timer.get() < 1.5){ 
          intakeSubsystem.setIntakeSpeed(0.5);
          liftLaunchSubsystem.setFeedSpeed(0.5);

      // Move intake back ands stop feed motors
      } else if(timer.get() > 1.5 & timer.get() < 3) { 
        intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
        liftLaunchSubsystem.setFeedSpeed(0);
        
      // Put note into position
      } else if(timer.get() > 3  ){ 
        intakeSubsystem.setIntakeSpeed(0);
        liftLaunchSubsystem.launchPercentageSet(0);
      //  intakeSubsystem.setLEDTwinkle(100, 40, 0);
        isDone = true;
      }
    }

  }

  // Reset motors to default state
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();

    liftLaunchSubsystem.setFeedSpeed(0);
    intakeSubsystem.setIntakeSpeed(0);
    liftLaunchSubsystem.launchPercentageSet(0);
    intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE); 
  }

  // Returns true once command is done
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
