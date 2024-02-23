// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.util.CommandXboxControllerExpand;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem;
import frc.robot.subsystems.LiftLaunchSubsystem.ShooterPos;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.ControlLift;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeMotor;
import frc.robot.commands.LaunchCMD;
import frc.robot.commands.LiftCMD;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Create a new DrivetrainSubsystem
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxControllerExpand m_driverController =
      new CommandXboxControllerExpand(OperatorConstants.kDriverControllerPort);
  private final CommandXboxControllerExpand m_driverController1 =
      new CommandXboxControllerExpand(OperatorConstants.kDriverControllerPort1);

  // Subsystems
  private final LiftLaunchSubsystem liftLaunchSubsystem = new LiftLaunchSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(liftLaunchSubsystem);

  // Create a new SlewRateLimiter with a maximum rate of 15
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(15);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Set Drive Controls
     drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(m_driverController.getRightX())),
            () -> m_driverController.getPOV()
    )); 
    // Configure the trigger bindings
    configureBindings();

    // Configure Autos Created by PathPlanner
    configureAutos();

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("None", getAutonomousCommand());
  }


  private void configureBindings() {
    //Zero the Gyroscope
    m_driverController.start().whileTrue(new InstantCommand(drivetrain::zeroGyroscope));

    //Intake Control
    //m_driverController1.a().onTrue(new IntakeCommand(intakeSubsystem, 30)).whileTrue(new IntakeMotor(0.30, intakeSubsystem));
    //m_driverController1.b().onTrue(new IntakeCommand(intakeSubsystem, 100)).whileTrue(new IntakeMotor(0.30, intakeSubsystem));

    m_driverController1.a().whileTrue(new ControlIntake(intakeSubsystem, IntakePos.Intake));
    m_driverController1.b().whileTrue(new ControlIntake(intakeSubsystem, IntakePos.Store));

    //m_driverController1.x().whileTrue(new IntakeMotor(-0.5, intakeSubsystem));

    m_driverController1.rightBumper().whileTrue(new LaunchCMD(liftLaunchSubsystem, 0.8));
    m_driverController1.leftBumper().whileTrue(new LaunchCMD(liftLaunchSubsystem, 1, 0.8));
    
    //m_driverController1.y().whileTrue(new LiftCMD(liftLaunchSubsystem, 55));
    //m_driverController1.x().whileTrue(new LiftCMD(liftLaunchSubsystem, 30));

    m_driverController1.y().whileTrue(new ControlLift(liftLaunchSubsystem, ShooterPos.Amp));
    m_driverController1.x().whileTrue(new ControlLift(liftLaunchSubsystem, ShooterPos.Speaker));
    m_driverController1.start().whileTrue(new ControlLift(liftLaunchSubsystem, ShooterPos.Store));

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

  /**
   *  Configure the autonomous commands. 
   *  See more ways to program paths using PathPanner's example: {@link https://github.com/mjansen4857/pathplanner/blob/b61ba22f5b24e395218d88abe363c8c669b6bdfe/examples/java/src/main/java/frc/robot/RobotContainer.java#L63}
   */
  private void configureAutos() {
    // Register named commands
    /* 
     * These will run specified commands named within a PathPlanner auto, for example "Print Hello" is named in "Example Auto".
     * This powerful tool allows us to program an entire auto within PathPlanner and run commands at anytime during the sequence.
     * If you want to add a command, simply place it where Commands.print("hello") is located.
     * 
     * Markers are also located within the PathPlanner, but these are only placed in paths, these can also run a command.
     */
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("intakeup", Commands.print("Intake Up"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));


    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    /* "Example Auto" is file to be deployed on the RoboRIO, This is created by using the PathPlanner Software */
    SmartDashboard.putData("Auto 3", new PathPlannerAuto("Auto 3"));

    /* See more ways to program paths using PathPanner's example located on GitHub */
    /* Although most of the commands and paths are setup here, we also need to make sure that the DrivetrainSubsystem can handle PathPlanner */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous 
    return autoChooser.getSelected();
  }
}
