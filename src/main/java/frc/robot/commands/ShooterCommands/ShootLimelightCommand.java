// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import com.limelight.LimelightHelpers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;

public class ShootLimelightCommand extends Command {
  private LiftLaunchSubsystem liftLaunchSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private double deployAngle, shooterSpeed, feedSpeed, distanceToSpeaker, lastXValue;
  private boolean isDone;
  private Timer timer;
  private LinearFilter filter;

  // Shoot using limelight
  public ShootLimelightCommand(LiftLaunchSubsystem liftLaunchSubsystem, IntakeSubsystem intakeSubsystem) {
    this.liftLaunchSubsystem = liftLaunchSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem, liftLaunchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    isDone = false;
    intakeSubsystem.setDeploySet(20);

    LimelightHelpers.setPipelineIndex("April_tag", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    alignWithLimelight();

    liftLaunchSubsystem.liftSet(deployAngle);
    liftLaunchSubsystem.setLaunchSpeed(shooterSpeed);

    if(timer.get() > 3 && timer.get() < 5){
      liftLaunchSubsystem.setFeedSpeed(feedSpeed);
    } else if(timer.get() > 5){
      isDone = true;
    }
  }

   public void alignWithLimelight(){
    if( LimelightHelpers.getTV("")){
        distanceToSpeaker = filter.calculate(Math.abs(LimelightHelpers.getCameraPose3d_TargetSpace("").getZ()));
        lastXValue = distanceToSpeaker;
    } else {
        distanceToSpeaker = lastXValue;
        filter.reset();
    }

    // Linear Regression (y = mx + b)
    shooterSpeed = (2 - 1)/(2- 1) * distanceToSpeaker + 0;
    deployAngle = (2 - 1)/(2- 1) * distanceToSpeaker + 0;
    feedSpeed = 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    liftLaunchSubsystem.liftSet(60); // TODO: change angle
    intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
    liftLaunchSubsystem.setFeedSpeed(0);
    liftLaunchSubsystem.setLaunchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
