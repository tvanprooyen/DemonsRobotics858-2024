// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.LiftCMD;
import frc.robot.commands.DriveCommand;
// import frc.robot.commands.LaunchCMD;
import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.LiftLaunchSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;


public class RobotContainer {

  // private final LiftLaunchSubsystem liftLaunchSubsystem = new LiftLaunchSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(5);

  private final XboxController  controller1 = new XboxController (0);
  private final XboxController controller2 = new XboxController(1);

  public RobotContainer() {
 
    drivetrain.register();

    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(controller1.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(controller1.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(controller1.getRightX())) //(-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) + 
    ));
    
  }


  private static double deadband(double value, double deadband) {
      if (Math.abs(value) > deadband) {
          if (value > 0.0) {
              return (value - deadband) / (1.0 - deadband);
          } else {
              return (value + deadband) / (1.0 - deadband);
          }
      } else {
          return 0.0;
      }
  }

  private static double modifyAxis(double value) {
      // Deadband
      value = deadband(value, 0.05);

      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }*/

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

}
