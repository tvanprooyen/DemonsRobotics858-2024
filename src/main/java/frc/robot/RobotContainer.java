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
//import frc.robot.commands.LiftCMD;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LaunchCMD;
import frc.robot.commands.LiftCMD;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;
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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;


public class RobotContainer {

  private final LiftLaunchSubsystem liftLaunchSubsystem = new LiftLaunchSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(5);

  private final XboxController  controller1 = new XboxController (0);
  private final XboxController controller2 = new XboxController(1);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();;

  public RobotContainer() {


    drivetrain.register();

    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(controller1.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(controller1.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(controller1.getRightX())) //(-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) + 
    ));

    configureBindings();
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

  private void configureBindings() {
    //Intake Control
    new JoystickButton(controller1, 2).whileTrue(new IntakeCommand(intakeSubsystem, 0.3));
    
    //Launch Control
    new JoystickButton(controller1, 6).whileTrue(new LaunchCMD(liftLaunchSubsystem, 1));
    
    //Feed Control
    new JoystickButton(controller1, 5).whileTrue(new LaunchCMD(liftLaunchSubsystem, 0.5, 0));

    //Shooter Lift Control
    //new JoystickButton(controller1, 3).onTrue(new LiftCMD(liftLaunchSubsystem, 90, 0));
  }

}
