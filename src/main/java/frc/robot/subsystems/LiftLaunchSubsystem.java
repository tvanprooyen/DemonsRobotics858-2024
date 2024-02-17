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
     private SlewRateLimiter liftLimiter = new SlewRateLimiter(1);
     private ShooterPos shooterPos;
     private Timer speakerTimer, ampTimer;

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

     public ShooterPos getShooterPose(){
        return this.shooterPos;
     }

     //launch things
     public void setLaunchSpeed(double launchSpeed){
         this.launchSpeed = launchSpeed;
     }

     private double getLaunchSpeed(){
         return this.launchSpeed;
     }


     //feed things
     public void setFeedSpeed(double feedSpeed){
         this.feedSpeed = feedSpeed;
     }

     private double getFeedSpeed(){
         return this.feedSpeed;
     }

     @Override
     public void periodic(){
       
        double liftValue = liftPID.calculate(getLiftPosition(), getLiftSet());
        double ShooterSet = 0;
        double tempFeedSpeed = getFeedSpeed();
        double tempLaunchSpeed = getLaunchSpeed();

        SmartDashboard.putNumber("LiftEncoder", getLiftPosition());
        SmartDashboard.putNumber("lift encoder", getLiftSet());
        SmartDashboard.putNumber("LiftPID", liftValue);

        liftValue = MathUtil.clamp(liftValue, -0.4, 0.4);

         /*if( getLiftPosition() > 60 || getLiftPosition() < 0 ){
             liftValue = 0;
         }
        mLift2.set(liftLimiter.calculate(liftValue));*/

        if(getLaunchSpeed() == 0){
            //Amp Sequence
            if(getLiftSet() == 70 && ampTimer.get() == 0){
                ampTimer.start();
                intakeSubsystem.setIntakePose(IntakePos.Amp);
            }

            if(ampTimer.get() > 8){
                ampTimer.stop();
                ampTimer.reset();
            }else
            if(ampTimer.get() > 7){
                tempFeedSpeed = 0;
                setShooterPose(ShooterPos.Store);
                intakeSubsystem.setIntakePose(IntakePos.Store);
            }else
            if(ampTimer.get() > 6){
                tempFeedSpeed = 0.8;
            }else
            if(ampTimer.get() > 2){
                setShooterPose(ShooterPos.Amp);
            }

            //Speaker Sequence
            if(getLiftSet() == 50 && speakerTimer.get() == 0){
                speakerTimer.start();
                intakeSubsystem.setIntakePose(IntakePos.Speaker);
            }

            if(speakerTimer.get() > 8){
                speakerTimer.stop();
                speakerTimer.reset();
            }else
            if(speakerTimer.get() > 7){
                tempFeedSpeed = 0;
                setShooterPose(ShooterPos.Store);
                intakeSubsystem.setIntakePose(IntakePos.Store);
            }else
            if(speakerTimer.get() > 6){
                tempFeedSpeed = 0.8;
            }else
            if(speakerTimer.get() > 2){
                setShooterPose(ShooterPos.Speaker);
            }

            
        switch(getShooterPose()){
                case Store:
            ShooterSet = 20;
            tempLaunchSpeed = 0;
            break;
                case Amp:
            ShooterSet = 70;
            tempLaunchSpeed = 0.6;
            break;
                case Speaker:
            ShooterSet = 50;
            tempLaunchSpeed = 0.8;
            break;
        }
    }
        mLaunch1.set(launchLimiter.calculate(getLaunchSpeed()));
        mLaunch2.set(launchLimiter.calculate(-getLaunchSpeed())); 
        
        mFeed1.set(-getFeedSpeed());
        mFeed2.set(getFeedSpeed());
     }

 }