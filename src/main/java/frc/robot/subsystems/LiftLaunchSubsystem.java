package frc.robot.subsystems;

 import javax.lang.model.util.ElementScanner14;

import com.revrobotics.AbsoluteEncoder;
 import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
 import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;

 public class LiftLaunchSubsystem extends SubsystemBase{

    public enum ShooterPos{
        Store,
        Amp,
        Speaker
    }

    // lift varables
     private final CANSparkMax mLift1 = new CANSparkMax(20, MotorType.kBrushless);
     private final CANSparkMax mLift2 = new CANSparkMax(21, MotorType.kBrushless);
     private AbsoluteEncoder liftEncoder;
     private double liftSet;
     private PIDController liftPID;
     private SlewRateLimiter liftLimiter = new SlewRateLimiter(0.25);
     private ShooterPos shooterPos;
     private Timer speakerTimer, ampTimer, TimeOut, feedTimer; //Set New Timers in Class

    // launch variables don't change
     private final CANSparkMax mLaunch1 = new CANSparkMax(23, MotorType.kBrushless);
     private final CANSparkMax mLaunch2 = new CANSparkMax(24, MotorType.kBrushless);
     private double launchSpeed;
     private SlewRateLimiter launchLimiter = new SlewRateLimiter(0.3);

    // feed variables also don't change
     private final CANSparkMax mFeed1 = new CANSparkMax(25, MotorType.kBrushless);
     private final CANSparkMax mFeed2 = new CANSparkMax(26, MotorType.kBrushless);
     private double feedSpeed;

     private IntakeSubsystem intakeSubsystem;

     public LiftLaunchSubsystem(){
     //default speeds
         this.launchSpeed = 0;
         this.feedSpeed = 0;
         mLaunch2.setInverted(true);
         

     //default lift
         liftPID = new PIDController(0.01, 0, 0);
         liftEncoder = mLift2.getAbsoluteEncoder(Type.kDutyCycle);
         this.liftSet =  liftEncoder.getPosition() * 360;

         mLift1.follow(mLift2, true);

         this.shooterPos = ShooterPos.Store;

         this.ampTimer = new Timer();
         this.speakerTimer = new Timer();

         this.TimeOut = new Timer();

         this.feedTimer = new Timer();

        SmartDashboard.putNumber("LiftEncoder", getLiftPosition());
        SmartDashboard.putNumber("lift encoder", getLiftSet());
        //SmartDashboard.putNumber("LiftPID", liftValue);
        SmartDashboard.putNumber("Shooter Timer", speakerTimer.get());

     }


     ///lift things
     //get position
     public double getLiftPosition(){
         return liftEncoder.getPosition() * 360;
     }

     //set PID position
     public void liftSet(double liftSet){
         this.liftSet = liftSet;
     }

     //get PID position
     public double getLiftSet(){
         return this.liftSet;
     }

     public boolean isInPosition(){
        return getLiftSet() - 5 < getLiftPosition() && getLiftSet() + 5 > getLiftPosition();
     }

     public void setShooterPose(ShooterPos SHOOTERPOS){
        this.shooterPos = SHOOTERPOS;
     }

     /**
      * Gets the Current Shooter Pose
      * @return ShooterPos
      */
     public ShooterPos getShooterPose(){
        return this.shooterPos;
     }

     /**
      * Sets the Launch Speed
      * @param launchSpeed -1 thru 1 Motor Speed(ex: 0.5 = %50)
      */
     public void setLaunchSpeed(double launchSpeed){
         this.launchSpeed = launchSpeed;
     }

     /**
      * Gets the current launch speed
      * @return -1 thru 1 Motor Speed(ex: 0.5 = %50)
      */
     private double getLaunchSpeed(){
         return this.launchSpeed;
     }


     /**
      * Sets the Feed Speed
      * @param feedSpeed -1 thru 1 Motor Speed(ex: 0.5 = %50)
      */
     public void setFeedSpeed(double feedSpeed){
         this.feedSpeed = feedSpeed;
     }

     private double getFeedSpeed(){
         return this.feedSpeed;
     }

     /* public double getSpeakerTimer(){
        return this.speakerTimer.get();
     } */ //No need for this

    private double getIsRunningTol() {
        return 3;
    }

    public boolean isRunning() {
        double tolerance = getIsRunningTol(); //In Deg Angle
        return isRunning(tolerance, true);
    }

    public boolean isRunning(boolean CheckMotors) {
        double tolerance = getIsRunningTol(); //In Deg Angle
        return isRunning(tolerance, CheckMotors);
    }

    public boolean isRunning(double tolerance) {
        return isRunning(tolerance, true);
    }

    public boolean isRunning(double tolerance, boolean CheckMotors) {
        boolean running = liftEncoder.getVelocity() < 0.01 && 
        (
            getLiftSet() + tolerance > getLiftPosition() && 
            getLiftSet() - tolerance < getLiftPosition()
        );

        if(CheckMotors) {
            running = running  &&
            (
                getLaunchSpeed() == 0 &&
                getFeedSpeed() == 0
            );
        }

        return running;
    }

     @Override
     public void periodic(){

        /* 
            Place Sequences Here
            Use Only Get and Set Methods
        */
        

        switch (getShooterPose()) {
            case Store:
                //SQ:1: Set Launch and Feed Motor to Zero, Lift Note Launcher To Set Angle | Do this until angle is satifactory
                //SQ:2: Trigger Intake to Storeage Pose | Do this if the intake isn't running

                double liftAngle = 50;
                //double intakeAngle = 40; // the current value is wrong //NO NEED FOR THIS ;)

                if(getLiftPosition() < liftAngle + 1 && getLiftPosition() > liftAngle - 1) {
                    //(SQ:1)
                    setLaunchSpeed(0);
                    setFeedSpeed(0);
                    liftSet(liftAngle);
                } else if(intakeSubsystem.getIntakePose() != IntakePos.Store) { //intakeSubsystem.getDeploySet() < intakeAngle + 1 && intakeSubsystem.getDeploySet() > intakeAngle - 1
                    //(SQ:2)
                    intakeSubsystem.setIntakePose(IntakePos.Store);
                }
                
                break;

            case Amp:
                 liftAngle = 50; // TODO the current value is wrong
                 double shooterVel = 0.8; // TODO the current value is wrong
                 double FeedSpeed = 0.5; // TODO the current value is wrong

                if(intakeSubsystem.getIntakePose() != IntakePos.Amp) {
                    intakeSubsystem.setIntakePose(IntakePos.Amp);
                } else if(!intakeSubsystem.isRunning() && getLiftPosition() < liftAngle + 1 && getLiftPosition() > liftAngle - 1){
                    liftSet(liftAngle);

                } else if(getLaunchSpeed() < shooterVel || TimeOut.get() < 60){
                    if(TimeOut.get() == 0){
                        TimeOut.start();
                    }
                    setLaunchSpeed(shooterVel);

                } if(feedTimer.get() < 3){
                    if(feedTimer.get() == 0){
                        feedTimer.start();
                    }
                    setFeedSpeed(FeedSpeed);
                }

               break;
            case Speaker:

               break;
        
            default:
                break;
        }

        //Then Run Motors
        runMotors();
     }


     private void runMotors() {
        double liftValue = liftPID.calculate(getLiftPosition(), getLiftSet());

        liftValue = MathUtil.clamp(liftValue, -0.4, 0.4);

         if( getLiftPosition() > 60 || getLiftPosition() < 0 ){
             liftValue = 0;
         }
        mLift2.set(liftLimiter.calculate(liftValue));

        mLaunch1.set(launchLimiter.calculate(getLaunchSpeed()));
        mLaunch2.set(-launchLimiter.calculate(getLaunchSpeed()));

        mFeed1.set(-getFeedSpeed());
        mFeed2.set(getFeedSpeed());
     }

 }