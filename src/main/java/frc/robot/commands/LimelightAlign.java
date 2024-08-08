// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.limelight.LimelightHelpers;
import com.limelight.LimelightHelpers.LimelightResults;
import com.limelight.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class LimelightAlign extends Command {
  /** Creates a new LimelightAlign. */
  private DrivetrainSubsystem drive;
  private String limelightName;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private boolean alignTranslationally;
  private double rotationalError,rotationAdjust,translationalError,translationalAdjust,robotAngle,translationXPercent,translationYPercent;

  public LimelightAlign(DrivetrainSubsystem drive, String limelightName, 
  DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, boolean alignTranslationally ){
    this.drive = drive;
    this.limelightName = limelightName;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.alignTranslationally = alignTranslationally;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightResults result = LimelightHelpers.getLatestResults("limelight-front");
  
    if( limelightName == "april" && LimelightHelpers.getTV("limelight-front")) {
      for( LimelightTarget_Fiducial target : result.targetingResults.targets_Fiducials) {
        if( target.fiducialID == 4 || target.fiducialID == 7){
          rotationalError = -target.tx;
        }
    }
    } else {
      rotationalError = -LimelightHelpers.getTX(limelightName);
    }
    rotationAdjust = -0.05 * rotationalError;

    translationalError = LimelightHelpers.getTY(limelightName);

    Rotation2d autoRotate = drive.getRotation();

    if( alignTranslationally ){
      translationalAdjust = 0.04 * translationalError;
      autoRotate = new Rotation2d();
    } else {
      translationalAdjust = 0;
    }

    robotAngle = drive.getRotationInDeg();
    if(robotAngle < 0){
      robotAngle = 360 + robotAngle;
    }

    SmartDashboard.putNumber("current angle", robotAngle);
    translationXPercent = translationXSupplier.getAsDouble();
    translationYPercent = translationYSupplier.getAsDouble();
   
    if( translationXPercent <= 0.1) translationXPercent = 0;
    if( translationYPercent <= 0.1) translationYPercent = 0;

    // SmartDashboard.putNumber("")
    SmartDashboard.putNumber("x adjust", translationalAdjust * Math.cos(Math.toRadians(robotAngle)));
    SmartDashboard.putNumber("y adjust", translationalAdjust * Math.sin(Math.toRadians(robotAngle)));


    drive.driveFieldRelative(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              translationalAdjust * Math.cos(Math.toRadians(robotAngle)) +  translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              translationalAdjust * Math.sin(Math.toRadians(robotAngle)) + translationYPercent *  DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              rotationAdjust,
              autoRotate
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
