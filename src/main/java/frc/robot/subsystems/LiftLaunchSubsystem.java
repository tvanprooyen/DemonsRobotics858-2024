// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.AbsoluteEncoder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LiftLaunchSubsystem extends SubsystemBase{
//     private final CANSparkMax mLift1 = new CANSparkMax(20, MotorType.kBrushless);
//     private final CANSparkMax mLift2 = new CANSparkMax(21, MotorType.kBrushless);
//     private AbsoluteEncoder liftEncoder;
//     private double liftSet;
//     private double liftSpeed;
//     private double liftError;
//     private double liftSetBuffer;
//     private PIDController liftPID;
//     private boolean shouldBuffer;

//     private final CANSparkMax mLaunch1 = new CANSparkMax(23, MotorType.kBrushless);
//     private final CANSparkMax mLaunch2 = new CANSparkMax(24, MotorType.kBrushless);
//     private double launchSpeed;

//     private final CANSparkMax mFeed1 = new CANSparkMax(25, MotorType.kBrushless);
//     private final CANSparkMax mFeed2 = new CANSparkMax(26, MotorType.kBrushless);
//     private double feedSpeed;

//     public LiftLaunchSubsystem(){
//     //default speeds
//         this.liftSpeed = 0;
//         this.launchSpeed = 0;
//         this.feedSpeed = 0;

//         this.shouldBuffer = true;

//     //default rotation/lift
//         liftPID = new PIDController(0.015, 0, 0);
//         this.liftSet = Math.toRadians(0);

//         liftEncoder = mLift1.getAbsoluteEncoder(Type.kDutyCycle);
//         this.liftError = 0;
//     }


//     ///lift things
//     //get position
//     public double getLiftPosition(){
//         return liftEncoder.getPosition();
//     }

//     //set PID position
//     public void liftSet(double liftSet){
//         this.liftSet = liftSet;
//     }

//     //get PID position
//     public double getLiftSet(){
//         return this.liftSet;
//     }

//     //lift PID error
//     public void setLiftError(double liftError){
//         this.liftError = liftError;
//     }

//     public double getLiftError(){
//         return this.liftError;
//     }

//     //tolerance within set position
//     public boolean isLiftInPosition(){
//         double liftTol = 1; //TODO need to adjust with testing

//         return (getLiftPosition() > (liftSet - liftTol)) && (getLiftPosition() < (liftSet + liftTol));
//     }

//     //sets PID buffer to save position
//     public void setLiftBuffer(double liftSetBuffer){
//         this.liftSetBuffer = liftSetBuffer;
//     }

//     public double getLiftBuffer(){
//         return this.liftSetBuffer;
//     }


//     //launch things
//     public void setLaunchSpeed(double launchSpeed){
//         this.launchSpeed = launchSpeed;
//     }

//     private double getLaunchSpeed(){
//         return this.launchSpeed;
//     }


//     //feed things
//     public void setFeedSpeed(double feedSpeed){
//         this.feedSpeed = feedSpeed;
//     }

//     private double getFeedSpeed(){
//         return this.feedSpeed;
//     }

//     @Override
//     public void periodic(){
//         mLaunch1.set(getLaunchSpeed());
//         mLaunch2.set(getLaunchSpeed());     
        
//         mFeed1.set(getFeedSpeed());
//         mFeed2.set(getFeedSpeed());
//     }

// }