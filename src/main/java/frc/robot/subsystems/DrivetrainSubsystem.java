package frc.robot.subsystems;

import java.util.function.Supplier;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SDSConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final PIDController RotatePID;

    private final PIDController OdoXPID;

    private final PIDController OdoYPID;

    private final SwerveDriveOdometry odometry;

    public final Pigeon2 gyroscope = new Pigeon2(SDSConstants.DRIVETRAIN_PIGEON_ID);

    private final Field2d fieldWG = new Field2d();

    private boolean RotateLock;

    private double RotateSet, XSet, YSet;

    private final ShuffleboardTab fieldSB = Shuffleboard.getTab("Field");

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
        /* new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroscope.getFusedHeading())); */

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SDSConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                SDSConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                SDSConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                SDSConstants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SDSConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                SDSConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                SDSConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                SDSConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SDSConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                SDSConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                SDSConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                SDSConstants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                SDSConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                SDSConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                SDSConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                SDSConstants.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(getGyroYaw()),
            new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

        //fieldWG.setRobotPose(new Pose2d(odometry.getPoseMeters().getY(), odometry.getPoseMeters().getX(), getRotation()));

        //updateField2d();
        //fieldSB.add("Robot Pose", fieldWG);

        RotatePID = new PIDController(0.01, 0.00, 0.00);
        RotatePID.enableContinuousInput(-180.0f,  180.0f);
        RotatePID.setTolerance(2);

        OdoXPID = new PIDController(0.3, 0.00, 0.0);
        OdoYPID = new PIDController(0.3, 0.00, 0.0);

        RotateSet = 0;

        this.RotateLock = false;

        //gyroscope.setYaw(0.0);
        
    }

    public void updateField2d() {
        fieldWG.setRobotPose(new Pose2d(odometry.getPoseMeters().getY(), odometry.getPoseMeters().getX(), getRotation()));
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroYaw()), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public double getRotationInDeg() {
        return getRotation().getDegrees();
    }

    public Pose2d robotPose() {
        return new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0));
    }


    public SwerveModuleState[] robotModuleStates(){
        return kinematics.toSwerveModuleStates(chassisSpeeds);
    }

    public double getGyroYaw() {
        return 0.0;
        // return gyroscope.getYaw().getValueAsDouble();
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public Pose2d getPos() {
        return odometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }


    public void setPIDRotateValue(double RotateSet) {
        this.RotateSet = RotateSet;
    }

    public double getPIDRotateValue() {
        return this.RotateSet;
    }

    public boolean getRotateLock(){
        return this.RotateLock;
    }

    public double rotatePIDCalculation() {
        double futureRotatePID = 
            MathUtil.clamp(
                RotatePID.calculate(
                    getRotation().getDegrees(), RotateSet),
             -1, 1);

        return futureRotatePID;
    }

    public void setPIDXValue(double XSet) {
        this.XSet = XSet;
    }

    public double getPIDXValue() {
        return this.XSet;
    }

    public void setPIDYValue(double YSet) {
        this.YSet = YSet;
    }

    public double getPIDYValue() {
        return this.YSet;
    }

    public double XPIDCalculation() {
        return OdoXPID.calculate(odometry.getPoseMeters().getX(), XSet);
    }

    public double YPIDCalculation() {
        return OdoXPID.calculate(odometry.getPoseMeters().getY(), YSet);
    }

    public void setRotateLock(boolean RotateLock) {
        this.RotateLock = RotateLock;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroYaw()), 
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }, 
            pose
        );
    }

    @Override
    public void periodic() {

        //rotatePIDCalculation();

        dashboard();

        odometry.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void setModuleStatesInverse(SwerveModuleState[] states) {
        frontLeftModule.set(-states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(-states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(-states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(-states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    private void dashboard() {
        SmartDashboard.putNumber("Rotate Set Point", getPIDRotateValue());
        SmartDashboard.putNumber("Rotate PID Calc", rotatePIDCalculation());
    }
}