 package frc.robot.commands;

 import frc.robot.subsystems.LiftLaunchSubsystem;

 public class LiftCMD {
     private LiftLaunchSubsystem liftLaunchSubsystem;
     private double liftSet;
     private double liftSpeed;
     private int placeholder;
     private boolean isFinished;
     //private boolean shouldBuffer;

     //TODO insert intake into both command things below
     
     //
     public LiftCMD(LiftLaunchSubsystem liftLaunchSubsystem, double liftSet, double liftSpeed/*, boolean shouldBuffer*/){
         this.liftLaunchSubsystem = liftLaunchSubsystem;
         this.liftSet = liftSet;
         this.liftSpeed = liftSpeed;
         this.isFinished = false;

         //this.shouldBuffer = true;
     }

     public LiftCMD(LiftLaunchSubsystem liftLaunchSubsystem, double liftSet, double liftSpeed, int placeholder){
         this.liftLaunchSubsystem = liftLaunchSubsystem;
         this.liftSet = liftSet;
         this.liftSpeed = 0;
         this.isFinished = false;
     }

     /*@Override
     public boolean isFinished(){
         return liftLaunchSubsystem.isLiftInPosition();
     } */
 }
