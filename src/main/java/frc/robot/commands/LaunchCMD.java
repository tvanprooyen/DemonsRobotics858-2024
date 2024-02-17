 package frc.robot.commands;

 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftLaunchSubsystem;
 
 public class LaunchCMD extends Command {
     private LiftLaunchSubsystem liftLaunchSubsystem;
     private double launchSpeed;
     private double feedSpeed;
     
     public LaunchCMD(LiftLaunchSubsystem liftLaunchSubsystem, double launchSpeed){
         this.liftLaunchSubsystem = liftLaunchSubsystem;
         this.launchSpeed = launchSpeed;
     }

     public LaunchCMD(LiftLaunchSubsystem liftLaunchSubsystem, double feedSpeed, double launchSpeed){
        this.liftLaunchSubsystem = liftLaunchSubsystem;
        this.feedSpeed = feedSpeed;
        this.launchSpeed = launchSpeed;
     }

     @Override 
    public void execute(){
        liftLaunchSubsystem.setLaunchSpeed(launchSpeed);
        liftLaunchSubsystem.setFeedSpeed(feedSpeed);
    }

     
    @Override
    public void end(boolean interupted) {
        liftLaunchSubsystem.setLaunchSpeed(0);
        liftLaunchSubsystem.setFeedSpeed(0);
     }
 }
