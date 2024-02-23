package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakePos {
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
    private double DeploySet;

    // feed variables
    private CANSparkMax IntakeMotor;
    private double IntakeSpeed;
    private Timer timer;
    private DigitalInput limitSwitch1;

    private IntakePos intakePos;
    private LiftLaunchSubsystem llSS;

    public IntakeSubsystem(){

        // deploy settings
        DeployMotor = new CANSparkMax(30, MotorType.kBrushless);
        DeployEncoder = DeployMotor.getAbsoluteEncoder(Type.kDutyCycle);
        DeployPID = new PIDController(0.008 , 0.0001, 0.002);
        DeployPID.enableContinuousInput(0, 360);
        this.DeploySet = getDeployEncoder();  //This will get the lift position. Methods can be called from a constructor, no need to do > DeployEncoder.getPosition() * 360

        // feed settings
        IntakeMotor = new CANSparkMax(31, MotorType.kBrushless);
        IntakeMotor.setInverted(true);
        this.IntakeSpeed = 0;
        limitSwitch1 = new DigitalInput(0);
        timer = new Timer();

        this.intakePos = IntakePos.Store;

        SmartDashboard.putNumber("P Value", 0);
        SmartDashboard.putNumber("I Value", 0);
        SmartDashboard.putNumber("D Value", 0);
    }

    /**
    * Sets the LiftLaunch Subsystem from the Robot Container
    * @param llSS LiftLaunchSubsystem
    */
    public void setLiftLaunchSubsystem(LiftLaunchSubsystem llSS) {
        this.llSS = llSS;
    }

    // deploy functions
    public double getDeployEncoder() {
        return DeployEncoder.getPosition() * 360;
    }

    public void setDeploySet(double DeploySet) {
        this.DeploySet = DeploySet;
    }

    public double getDeploySet() {
        return this.DeploySet;
    }

    // feed functions
    public void setIntakeSpeed(double IntakeSpeed) {
        this.IntakeSpeed = IntakeSpeed;
    }

    public double getIntakeSpeed() {
        return this.IntakeSpeed;
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

    public boolean isRunning() {
        double tolerance = 3; //In Deg Angle
        return isRunning(tolerance);
    }

    public boolean isRunning(double tolerance) {
        return DeployEncoder.getVelocity() < 0.01 && 
        (
            getDeploySet() + tolerance > getDeployEncoder() && 
            getDeploySet() - tolerance < getDeployEncoder()
        ) &&
        (
            getIntakeSpeed() == 0
        ); //TODO CHECK MATH AND COMPARE
    }

    @Override
    public void periodic() {

        // DeployPID.setP(SmartDashboard.getNumber("P Value", 0.002));
        // DeployPID.setI(SmartDashboard.getNumber("I Value", 0));
        // DeployPID.setD(SmartDashboard.getNumber("D Value", 0));

        // deploy
        //double deployValue = DeployPID.calculate(getDeployEncoder(), getDeploySet());
        SmartDashboard.putNumber("encoder", getDeployEncoder());
        /* SmartDashboard.putNumber("IntakePID", deployValue);
        //deployValue = MathUtil.clamp(deployValue, -0.4, 0.4);
        DeployMotor.set(deployValue); */

        SmartDashboard.putNumber("Intake Timer", timer.get());

        double tempintakespeed = getIntakeSpeed();
        double IntakeDeploy = 0; //getDeploySet()
        double MaxDeploySpeed = 1;

        if(getIntakeSpeed() == 0) {
            if (getLimitSwitch() && timer.get() == 0) {
                timer.start();
                setIntakePose(IntakePos.Store);
            }

            if(timer.get() > 9.1) {
                timer.stop();
                timer.reset();
            } else if(timer.get() > 9) {
                llSS.setFeedSpeed(0);
                llSS.setLaunchSpeed(0);
            } else if(timer.get() > 8.5) {
                llSS.setFeedSpeed(-0.3);
                llSS.setLaunchSpeed(-0.2);
                setIntakePose(IntakePos.Store);
            } else if(timer.get() > 7.5) {
                setIntakePose(IntakePos.StoreFeed);
            } else if(timer.get() > 7) {
                setIntakePose(IntakePos.Feed);
                llSS.setFeedSpeed(0);
                //llSS.setLaunchSpeed(0);
            } else if(timer.get() > 5) {
                setIntakePose(IntakePos.Feed);
                llSS.setFeedSpeed(0.6);
                //llSS.setLaunchSpeed(0.2);
            } else if(timer.get() > 3) {
                setIntakePose(IntakePos.ReadyFeed);
            } 
            
            switch (getIntakePose()) {
                case Intake:
                    IntakeDeploy = 116;
                    MaxDeploySpeed = 0.3;
                    tempintakespeed = 0.7;
                    if ((getLimitSwitch() && getIntakeSpeed() > 0)) {
                        tempintakespeed = 0;
                    }
                    break;

                case Store:
                    IntakeDeploy = 45;
                    MaxDeploySpeed = 0.3;
                    tempintakespeed = 0;
                    break;

                case StoreFeed:
                    IntakeDeploy = 45;
                    MaxDeploySpeed = 0.3;
                    tempintakespeed = 0.3;
                    break;
            
                case Feed:
                    IntakeDeploy = 350;
                    tempintakespeed = 0.3;
                    break;
                case ReadyFeed:
                    //MaxDeploySpeed = 0.1;
                    IntakeDeploy = 0;
                    tempintakespeed = 0;
                break;
                case Amp:
                    MaxDeploySpeed = 0.1;
                    IntakeDeploy = 116;
                    tempintakespeed = 0;
                break;
                case Speaker:
                    MaxDeploySpeed = 0.1;
                    IntakeDeploy = 45;
                    tempintakespeed = 0;
                break;
            } 
        }
        

        /* if ((getLimitSwitch() && getIntakeSpeed() > 0)) {
            tempintakespeed = 0;
        }

        if (timer.get() >= 100 && timer.get() < 200) {
            tempintakespeed = 0;
        } else if (timer.get() > 10000) {
            timer.stop();
            timer.reset();
        } */

        double deployValue = DeployPID.calculate(getDeployEncoder(), IntakeDeploy);
        deployValue = MathUtil.clamp(deployValue, -MaxDeploySpeed, MaxDeploySpeed);
        DeployMotor.set(deployValue);

        IntakeMotor.set(tempintakespeed);

        /* if ((getLimitSwitch() && getIntakeSpeed() > 0)) {
            tempintakespeed = 0;
        } else {
            timer.stop();
            timer.reset();
        }

        if (getLimitSwitch()) {
            timer.start();
        }

        if (timer.get() >= 100 && timer.get() < 200) {
            tempintakespeed = 0;
        } else if (timer.get() > 200) {
            timer.stop();
            timer.reset();
        } */


    }

}