package frc.robot.subsystems;
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
    private double IntakeSpeed;
    private Timer timer;
    private Timer timer2;
    private Timer timer3;
    private DigitalInput limitSwitch1;

    private IntakePos intakePos;
    private LiftLaunchSubsystem llSS;

    public IntakeSubsystem(){

        // deploy settings
        DeployMotor = new CANSparkMax(30, MotorType.kBrushless);
        DeployEncoder = DeployMotor.getAbsoluteEncoder(Type.kDutyCycle);
        DeployPID = new PIDController(0.008 , 0.0000, 0.0001);
        //DeployPID = new ProfiledPIDController(0.008 , 0.00003, 0.00006, new TrapezoidProfile.Constraints(60, 40));
        DeployPID.enableContinuousInput(0, 360);
        this.DeploySet = 10;  //This will get the lift position. Methods can be called from a constructor, no need to do > DeployEncoder.getPosition() * 360

        // feed settings
        IntakeMotor = new CANSparkMax(31, MotorType.kBrushless);
        IntakeMotor2 = new CANSparkMax(32, MotorType.kBrushless);
        IntakeMotor2.follow(IntakeMotor, false);
        IntakeMotor.setInverted(false);
        this.IntakeSpeed = 0;
        limitSwitch1 = new DigitalInput(0);
        timer = new Timer();

        timer2 = new Timer();

        timer3 = new Timer();

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

    public void AutoIntake() {
        this.intakePos = IntakePos.Intake;
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
        double feedforward = 0;

        if(getIntakeSpeed() == 0) {

            // check for IR sensor and if the shooter is in position
           if (getLimitSwitch() && timer.get() == 0 && llSS.getShooterPose() == ShooterPos.Store) {
                setIntakePose(IntakePos.ReadyFeed); // get intake ready for feed

                if(!isRunningNoBelt()) {
                    timer.start();
                }
            }
            runIntakeAuto();

            // this is my change it may not work
            if (getLimitSwitch() && timer.get() == 0 && llSS.getShooterPose() == ShooterPos.Store) {
                setIntakePose(IntakePos.Store); // store intake after it collects the note

                // don't start feed system
               // if(!isRunningNoBelt()) {
               //     timer.start();
               // }
            }

            // automatic feeding system.  
            /*if(timer.get() > 2) { // End: store intake an stop the shooter.
                setIntakePose(IntakePos.Store);
                timer.stop();
                timer.reset();
                llSS.setFeedSpeed(0);
                llSS.setLaunchSpeed(0);
                //llSS.setLaunchSpeed(0);
            } if(timer.get() > 1.75) { // for 0.25 seconds run shooter backwards
                llSS.setFeedSpeed(-0.3);
                llSS.setLaunchSpeed(-0.3);
            } else if(timer.get() > 1.5) { // for 0.5 seconds run shooter forwards
                llSS.setFeedSpeed(0.1);
                llSS.setLaunchSpeed(0.1);
            } else if(timer.get() > 1) { // Start: for one second feed intake
                setIntakePose(IntakePos.Feed);
                //llSS.setFeedSpeed(0.8);
                //llSS.setLaunchSpeed(0.1); 
                //llSS.setLaunchSpeed(0.2);
            }*/

            /* if(timer.get() > 3.8) {
                timer.stop();
                timer.reset();
            } else if(timer.get() > 3.7) {
                llSS.setFeedSpeed(0);
                llSS.setLaunchSpeed(0);
            } else if(timer.get() > 3.5) {
                llSS.setFeedSpeed(-0.3);
                llSS.setLaunchSpeed(-0.2);
                setIntakePose(IntakePos.Store);
            } else if(timer.get() > 3) {
                setIntakePose(IntakePos.StoreFeed);
            } else if(timer.get() > 2) {
                setIntakePose(IntakePos.Feed);
                llSS.setFeedSpeed(0);
                llSS.setLaunchSpeed(0);
                //llSS.setLaunchSpeed(0);
            } else if(timer.get() > 1.8) {
                setIntakePose(IntakePos.Feed);
                llSS.setFeedSpeed(0.8);
                llSS.setLaunchSpeed(0.1);
                //llSS.setLaunchSpeed(0.2);
            } else if(timer.get() > 0.5) {
                setIntakePose(IntakePos.ReadyFeed);
            } */

             if(getIntakePose() != IntakePos.Intake && getIntakePose() != IntakePos.Amp) {
                timer2.stop();
                timer2.reset();

                timer3.stop();
                timer3.reset();
            }
            
            switch (getIntakePose()) {
                case Intake:

                    /* if(timer2.get() == 0) {
                        timer2.start();
                    } */

                    /* if(timer2.get() > 2) {
                        IntakeDeploy = 90;
                    } else if(timer2.get() > 1) {
                        IntakeDeploy = 60;
                    } */


                     IntakeDeploy = 99;

                    if(timer3.get() == 0) {
                        timer3.start();
                    } else if(timer3.get() < 1) {
                        IntakeDeploy = 60;
                    } else if(timer3.get() < 2) {
                        IntakeDeploy = 65;
                    }

                    MaxDeploySpeed = 0.1;
                    tempintakespeed = 0.5;
                    if ((getLimitSwitch() && timer2.get() == 0  && getIntakeSpeed() > 0 )) {
                        //tempintakespeed = 0;
                         timer2.start();
                    }

                    if(timer2.get() > 0) {
                        tempintakespeed = 0;
                    } else if(timer2.get() > 0.35) {
                        timer2.stop();
                        timer2.reset();
                    }

                    break;

                case PreIntake:
                    IntakeDeploy = 90;
                    MaxDeploySpeed = 0.1;
                    tempintakespeed = 0.95;
                    break;

                case Store:
                    IntakeDeploy = 35;
                    //MaxDeploySpeed = 0.6;
                    tempintakespeed = 0;
                    break;

                case StoreFeed:
                    IntakeDeploy = 45;
                    //MaxDeploySpeed = 0.4;
                    tempintakespeed = 0.5;
                    break;
            
                case Feed:
                    MaxDeploySpeed = 0.4;
                    IntakeDeploy = 10;
                    tempintakespeed = 0.6;
                    feedforward = -0.3;
                    break;
                case ReadyFeed:
                    IntakeDeploy = 10;
                    tempintakespeed = 0;
                break;
                case Amp:
                    MaxDeploySpeed = 0.1;
                    IntakeDeploy = 90;

                    if(timer3.get() == 0) {
                        timer3.start();
                    } else if(timer3.get() < 1) {
                        IntakeDeploy = 70;
                    } else if(timer3.get() < 2) {
                        IntakeDeploy = 75;
                    }

                    tempintakespeed = 0;
                break;
                case Speaker:
                    MaxDeploySpeed = 0.6;
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
        //deployValue = MathUtil.clamp(deployValue, -0.7, 0.7);

        deployValue = MathUtil.clamp(deployValue, -MaxDeploySpeed, MaxDeploySpeed);

        //DeployMotor.setIdleMode(IdleMode.kBrake);

        /* if(getDeployEncoder() >= 50 && deployValue > 0) {
            deployValue = MathUtil.clamp(deployValue, -0.1, 0.1);
            DeployMotor.setIdleMode(IdleMode.kBrake);
        } else {
            deployValue = MathUtil.clamp(deployValue, -0.7, 0.7);
            DeployMotor.setIdleMode(IdleMode.kCoast);
        } */



        DeployMotor.set(deployValue);// + feedforward);

        IntakeMotor.set(getIntakeSpeed() + tempintakespeed);
        //IntakeMotor2.set(getIntakeSpeed() + tempintakespeed); 

         /*DeployMotor.set(0);

        IntakeMotor.set(0);
        IntakeMotor2.set(0);*/

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

     // this might not work
     public void runIntakeAuto(){

        // check for IR sensor and if the shooter is in position
           if (getLimitSwitch() && timer.get() == 0 && llSS.getShooterPose() == ShooterPos.Store) {
                setIntakePose(IntakePos.ReadyFeed); // get intake ready for feed

                if(!isRunningNoBelt()) {
                    timer.start();
                }
            }

            // automatic feeding system.  
            if(timer.get() > 2) { // End: store intake an stop the shooter.
                setIntakePose(IntakePos.Store);
                timer.stop();
                timer.reset();
                llSS.setFeedSpeed(0);
                llSS.setLaunchSpeed(0);
                //llSS.setLaunchSpeed(0);
                //return true;
            } if(timer.get() > 1.75) { // for 0.25 seconds run shooter backwards
                llSS.setFeedSpeed(-0.3);
                llSS.setLaunchSpeed(-0.3);
            } else if(timer.get() > 1.5) { // for 0.5 seconds run shooter forwards
                llSS.setFeedSpeed(0.1);
                llSS.setLaunchSpeed(0.1);
            } else if(timer.get() > 1) { // Start: for one second feed intake
                setIntakePose(IntakePos.Feed);
                //llSS.setFeedSpeed(0.8);
                //llSS.setLaunchSpeed(0.1); 
                //llSS.setLaunchSpeed(0.2);
            }
    }

 }