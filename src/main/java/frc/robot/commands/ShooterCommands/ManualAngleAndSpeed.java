// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LaunchConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;

public class ManualAngleAndSpeed extends Command {
  private LiftLaunchSubsystem liftLaunchSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private double intakeAngle,shooterAngle,shooterSpeed;
  private double intakeSpeed, feedSpeed;

  /** Creates a new AmpShotCommand. */
  public ManualAngleAndSpeed(LiftLaunchSubsystem liftLaunchSubsystem, IntakeSubsystem intakeSubsystem, 
    double intakeAngle, double intakeSpeed, double shooterAngle, double feedSpeed, double shooterSpeed) {
      this.liftLaunchSubsystem = liftLaunchSubsystem;
      this.intakeSubsystem = intakeSubsystem;
      this.intakeAngle = intakeAngle;
      this.shooterAngle = shooterAngle;
      this.shooterSpeed = shooterSpeed;
      this.intakeSpeed = intakeSpeed;
      this.feedSpeed = feedSpeed;

      addRequirements(intakeSubsystem);
      addRequirements(liftLaunchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // -1 means we do not want to set an angle, and want it to be whatever it is currently
    if (shooterAngle != -1) {
      liftLaunchSubsystem.liftSet(shooterAngle);
    }
    if (intakeAngle != -1) {
      intakeSubsystem.setDeploySet(intakeAngle);
    }

    liftLaunchSubsystem.launchPercentageSet(shooterSpeed);
    intakeSubsystem.setIntakeSpeed(intakeSpeed);
      liftLaunchSubsystem.setFeedSpeed(feedSpeed);

      // intakeSubsystem.setLEDRGB(
      //   (int)Math.random() * 255, 
      //   (int)Math.random() * 255, 
      //   (int)Math.random() * 255
      // );
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Return everything back to default values
    liftLaunchSubsystem.liftSet(LaunchConstants.SHOOTER_HANDOFF_ANGLE);
    liftLaunchSubsystem.launchPercentageSet(0);
    intakeSubsystem.setIntakeSpeed(0);
    liftLaunchSubsystem.setFeedSpeed(0);

    // If shooterAngle is high, default the intake lower than usual
    if( shooterAngle > 80){
      intakeSubsystem.setDeploySet(330);
    } else {
      intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
