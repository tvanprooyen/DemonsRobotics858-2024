package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.limelight.LimelightHelpers;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;
    private final BooleanSupplier trackNote;

    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier,
            BooleanSupplier trackNote
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;
        this.trackNote = trackNote;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        int povPos = povSupplier.getAsInt();
        boolean isTracking = trackNote.getAsBoolean();

        //Allows the Robot to angle its self if the POV is pressed and with in the deadband of the axis
        if(rotationPercent < 0.05 && rotationPercent > -0.05) {
            if(povPos != -1) {
                //-360 flips the axis
                drivetrain.setPIDRotateValue(360 - povPos);
                drivetrain.setRotateLock(true);
            }
        } else {
            drivetrain.setRotateLock(false);
        }

        if(drivetrain.getRotateLock()) {
            rotationPercent = -drivetrain.rotatePIDCalculation();

            if(rotationPercent > 0.2) {
                rotationPercent = 0.2;
            } else if(rotationPercent < -0.2) {
                rotationPercent = -0.2;
            }
        }

        double rotationalError = -LimelightHelpers.getTX("");
        double rotationAdjust = -0.05 * rotationalError;

        double translationalError = LimelightHelpers.getTY("");
        double translationalAdjust = 0.06 * translationalError;

        double robotAngle = drivetrain.getRotationInDeg();
        
        double xAdjust = 0;
        double yAdjust = 0;
        double rotAdjust = 0;

        if(isTracking) {
            xAdjust = translationalAdjust * Math.cos(Math.toRadians(robotAngle)) +  translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            yAdjust = translationalAdjust * Math.sin(Math.toRadians(robotAngle)) + translationYPercent *  DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            rotAdjust = rotationAdjust;


            /* drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        rotAdjust + (rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                        drivetrain.getRotation()
                )
            ); */

            drivetrain.drive(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        -translationalAdjust,
                        0.0,
                        rotAdjust + (rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                        new Rotation2d()
                )
            );
        } else {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

}