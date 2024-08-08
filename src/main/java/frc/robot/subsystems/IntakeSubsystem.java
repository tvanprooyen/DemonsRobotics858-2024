package frc.robot.subsystems;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LiftLaunchSubsystem.ShooterPos;
import frc.robot.util.TimeOutTimer;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakePos {
        PreIntake,
        Intake,
        Store,
        StoreFeed,
        ReadyFeed,
        Feed,
        Amp,
        Speaker
    }

    // deploy variables
    private CANSparkMax DeployMotor;
    private AbsoluteEncoder DeployEncoder;
    private PIDController DeployPID;
    //private ProfiledPIDController DeployPID;
    private double DeploySet;

    // feed variables
    private CANSparkMax IntakeMotor, IntakeMotor2;
    private double intakeSpeedTop, intakeSpeedBottom;
    private DigitalInput limitSwitch1;
    private CANdle candle;
    private CANdleConfiguration config;

    private IntakePos intakePos;

    public IntakeSubsystem(){

        candle = new CANdle(15);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;

        candle.configAllSettings(config);

        candle.setLEDs(255,0,255);

        RainbowAnimation rainbow = new RainbowAnimation(1, 0.5, 82);
        candle.animate(rainbow);

        // deploy settings
        DeployMotor = new CANSparkMax(30, MotorType.kBrushless);
        DeployEncoder = DeployMotor.getAbsoluteEncoder(Type.kDutyCycle);
        DeployPID = new PIDController(0.008 , 0.0000, 0.0001);
        //DeployPID = new ProfiledPIDController(0.008 , 0.00003, 0.00006, new TrapezoidProfile.Constraints(60, 40));
        DeployPID.enableContinuousInput(0, 360);
        this.DeploySet = IntakeConstants.INTAKE_STORE_ANGLE;  //This will get the lift position. Methods can be called from a constructor, no need to do > DeployEncoder.getPosition() * 360

        // feed settings
        IntakeMotor = new CANSparkMax(31, MotorType.kBrushless);
        IntakeMotor2 = new CANSparkMax(32, MotorType.kBrushless);
       // IntakeMotor2.follow(IntakeMotor);
        this.intakeSpeedTop = 0;
        this.intakeSpeedBottom = 0;
        limitSwitch1 = new DigitalInput(0);

        this.intakePos = IntakePos.Store;

        SmartDashboard.putNumber("P Value", 0);
        SmartDashboard.putNumber("I Value", 0);
        SmartDashboard.putNumber("D Value", 0);
    }


    // deploy functions
    public double getDeployEncoder() {
        return DeployEncoder.getPosition() * 360;
    }

    public void setLEDRGB(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setLEDTwinkle(int r, int g, int b){
        Animation animate = new TwinkleAnimation(r,g,b);
        candle.animate(animate);
    }
    public void cancelLEDAnimation(){
        candle.animate(null);
    }
    

    public void setDeploySet(double DeploySet) {
        this.DeploySet = DeploySet;
    }

    public double getDeploySet() {
        return this.DeploySet;
    }

    public void setIntakeSpeed(double intakeSpeed){
        this.intakeSpeedTop = intakeSpeed;
        this.intakeSpeedBottom = intakeSpeed;
    }

    // feed functions
    public void setIntakeSpeedTop(double intakeSpeedTop) {
        this.intakeSpeedTop = intakeSpeedTop;
    }

    public double getIntakeSpeedTop() {
        return this.intakeSpeedTop;
    }

     public void setIntakeSpeedBottom(double intakeSpeedBottom) {
        this.intakeSpeedBottom = intakeSpeedBottom;
    }

    public double getIntakeSpeedBottom() {
        return this.intakeSpeedBottom;
    }

    public boolean getLimitSwitch() {
        return !limitSwitch1.get();
    }

    public boolean isInPose() {
        return getDeploySet() - 5 < getDeployEncoder() && getDeploySet() + 5 > getDeployEncoder();
    }

    public void setIntakePose(IntakePos INTAKEPOSE) {
        this.intakePos = INTAKEPOSE;
    }

    public IntakePos getIntakePose() {
        return this.intakePos;
    }

    public void AutoIntake() {
        this.intakePos = IntakePos.Intake;
    }

    public boolean isRunningNoBelt() {
        double tolerance = 3; //In Deg Angle
        return DeployEncoder.getVelocity() < 0.01 && 
        (
            getDeploySet() + tolerance > getDeployEncoder() && 
            getDeploySet() - tolerance < getDeployEncoder()
        );
    }

    

    @Override
    public void periodic() {
        double deployValue = DeployPID.calculate(getDeployEncoder(), getDeploySet());
        deployValue = MathUtil.clamp(deployValue, -0.4, 0.4);

        DeployMotor.set(deployValue);
        IntakeMotor.set(getIntakeSpeedTop());
        IntakeMotor2.set(getIntakeSpeedBottom());

        // System.out.println("Bottom Speed: " + getIntakeSpeedBottom());
        // System.out.println("Top Speed: " +getIntakeSpeedTop());
    }

 }