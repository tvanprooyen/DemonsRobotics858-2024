// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import com.limelight.LimelightHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LaunchConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;

public class ShooterTimedCommand extends Command {

  private LiftLaunchSubsystem liftLaunchSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private double deployAngle, shooterSpeed, feedSpeed, intakeAngle;
  private Timer timer;

  // Shooting to Speaker or Amp using inputted Values
  public ShooterTimedCommand(LiftLaunchSubsystem liftLaunchSubsystem, IntakeSubsystem intakeSubsystem, double deployAngle, double shooterSpeed, double feedSpeed, double intakeAngle, Timer timer) {
     this.liftLaunchSubsystem = liftLaunchSubsystem;
     this.intakeSubsystem = intakeSubsystem;
     this.feedSpeed = feedSpeed;
     this.deployAngle = deployAngle;
     this.intakeAngle = intakeAngle;
     this.shooterSpeed = shooterSpeed;
     this.timer = timer;
    
     addRequirements(intakeSubsystem, liftLaunchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    timer.start();
    intakeSubsystem.setDeploySet(intakeAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Lift up Shooter and Spin up motors
    if(timer.get() > 0 && timer.get() < 1.25){
     // intakeSubsystem.cancelLEDAnimation();
      //intakeSubsystem.setLEDRGB(255,0,255);
      liftLaunchSubsystem.liftSet(deployAngle);
      liftLaunchSubsystem.setLaunchSpeed(shooterSpeed);

    // Feed note forward
    } else if(timer.get() > 1.25 && timer.get() < 2){
    //  System.out.println("Feed Speed:" + feedSpeed);
      liftLaunchSubsystem.setFeedSpeed(feedSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    liftLaunchSubsystem.liftSet(LaunchConstants.LAUNCH_ANGLE); 
    intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
    liftLaunchSubsystem.setFeedSpeed(0);
    liftLaunchSubsystem.setLaunchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

