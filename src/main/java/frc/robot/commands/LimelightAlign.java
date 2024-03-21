// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class LimelightAlign extends Command {
  /** Creates a new LimelightAlign. */
  private DrivetrainSubsystem drive;
  private Limelight limelight;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private boolean alignTranslationally;
  private double rotationalError,rotationAdjust,translationalError,translationalAdjust,robotAngle,translationXPercent,translationYPercent;

  public LimelightAlign(DrivetrainSubsystem drive, Limelight limelight, 
  DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, boolean alignTranslationally ){
    this.drive = drive;
    this.limelight = limelight;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.alignTranslationally = alignTranslationally;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationalError = -limelight.GetX();
    rotationAdjust = -0.05 * rotationalError;

    translationalError = limelight.GetY();

    if( alignTranslationally ){
      translationalAdjust = 0.04 * translationalError;
    } else {
      translationalAdjust = 0;
    }

    robotAngle = drive.getRotationInDeg();
    translationXPercent = translationXSupplier.getAsDouble();
    translationYPercent = translationYSupplier.getAsDouble();

    if( translationXPercent <= 0.1) translationXPercent = 0;
    if( translationYPercent <= 0.1) translationYPercent = 0;


    drive.driveFieldRelative(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              translationalAdjust * Math.cos(Math.toRadians(robotAngle)) +  translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              translationalAdjust * Math.sin(Math.toRadians(robotAngle)) + translationYPercent *  DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              rotationAdjust,
              drive.getRotation()
        )
    );

     /* drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationalAdjust,
                        0.0,
                        rotationAdjust,
                        drive.getRotation()
                )
        ); */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
