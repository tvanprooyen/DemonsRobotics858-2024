package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.limelight.LimelightHelpers;
import com.limelight.LimelightHelpers.LimelightResults;
import com.limelight.LimelightHelpers.LimelightTarget_Fiducial;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final IntSupplier povSupplier;
    private final BooleanSupplier trackNote;
    private final BooleanSupplier trackSpeaker;

    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            IntSupplier povSupplier,
            BooleanSupplier trackNote,
            BooleanSupplier trackSpeaker
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.povSupplier = povSupplier;
        this.trackNote = trackNote;
        this.trackSpeaker = trackSpeaker;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        int povPos = povSupplier.getAsInt();
        boolean isTrackingNote = trackNote.getAsBoolean();
        boolean isTrackingSpeaker = trackSpeaker.getAsBoolean();


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

        double rotationalError = 0;
        double rotationAdjust = 0;

        double translationalError = 0;
        double translationalAdjust = 0;

        double robotAngle = drivetrain.getRotationInDeg();

        Rotation2d autoRotate = drivetrain.getRotation();

        //Base
        translationXPercent = (translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        translationYPercent = (translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        rotationPercent = (rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        if(isTrackingNote) {
            rotationalError = -LimelightHelpers.getTX("limelight");

            rotationAdjust = -0.05 * rotationalError;

            translationalError = LimelightHelpers.getTY("limelight");

            translationalAdjust = 0.04 * translationalError;

            if(robotAngle < 0){
                robotAngle = 360 + robotAngle;
            }

            //Track Note Profile
            translationXPercent += (translationalAdjust * Math.cos(Math.toRadians(robotAngle)));
            translationYPercent += (translationalAdjust * Math.sin(Math.toRadians(robotAngle)));
            rotationPercent += rotationAdjust;
            //autoRotate = new Rotation2d();

        } else if(isTrackingSpeaker) {
            
            LimelightResults result = LimelightHelpers.getLatestResults("limelight-front");
  
            if(LimelightHelpers.getTV("limelight-front")) {
                for( LimelightTarget_Fiducial target : result.targetingResults.targets_Fiducials) {
                    if( target.fiducialID == 4 || target.fiducialID == 7){
                        rotationalError = -target.tx;
                    }
                }
            }

            rotationAdjust = -0.08 * rotationalError;

            translationalError = LimelightHelpers.getTY("limelight-front");

            //Track Note Profile
            rotationPercent += rotationAdjust;
        }

        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationXPercent,
                    translationYPercent,
                    rotationPercent,
                    autoRotate
            )
        );
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