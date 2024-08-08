// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LaunchConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.CommandXboxControllerExpand;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LiftLaunchSubsystem;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LiftLaunchSubsystem.ShooterPos;
 import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDControl;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;
  // import frc.robot.commands.LiftCMD;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.AutoCommands.IntakeHandoffCommandAuto;
import frc.robot.commands.ShooterCommands.ShooterTimedCommand;
import frc.robot.commands.AutoCommands.ShooterTimedCommandAuto;
import frc.robot.commands.IntakeCommands.IntakeDeployCommand;
import frc.robot.commands.IntakeCommands.IntakeHandoffCommand;
import frc.robot.commands.ShooterCommands.ManualAngleAndSpeed;
import frc.robot.commands.ShooterCommands.ShootLimelightCommand;

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
  //private final LEDControl ledControl = new LEDControl();

  private final LiftLaunchSubsystem liftLaunchSubsystem = new LiftLaunchSubsystem();
  //private final Limelight limelightSubsystem = new Limelight();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Create a new SlewRateLimiter with a maximum rate of 15
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(15);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(15);
  private final Timer timer = new Timer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Set Subsystems for cross communication
  
    //Set Drive Controls
     drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftY())), // Axes are flipped here on purpose
            () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftX())),
            () -> rotLimiter.calculate(modifyAxis(m_driverController.getRightX())),
            () -> m_driverController.getMixPOV(),
            () -> m_driverController.rightTrigger().getAsBoolean(), //Track Note (Drive & Steer)
            () -> m_driverController.leftTrigger().getAsBoolean() //Track Speaker (Steer)
    )); 
    // Configure the trigger bindings
    configureBindings();

    // Configure Autos Created by PathPlanner
    configureAutos();

    autoChooser = AutoBuilder.buildAutoChooser();
    
    autoChooser.addOption("None", getAutonomousCommand());

    SmartDashboard.putData(autoChooser);
  }


  private void configureBindings() {

    //Driver (Start): Zero the Gyroscope
    m_driverController.start().whileTrue(new InstantCommand(drivetrain::zeroGyroscope));

    //Driver (Start): Zero the Gyroscope with 180 offset
    m_driverController.back().whileTrue(new InstantCommand(drivetrain::zeroGyroscope180));

    //All this has been moved to drive command, Having mutiple creates a conflicting drive command
     // Driver (A): Steer towards note
    /* m_driverController.a().whileTrue(new LimelightAlign(drivetrain, "limelight", 
    () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftY())), // Axes are flipped here on purpose
    () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftX())), false));

    // Driver (B): Drive towards note
    m_driverController.b().whileTrue(new LimelightAlign(drivetrain, "limelight", 
    () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftX())), // Axes are flipped here on purpose
    () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftY())),true)); */

    /* // Driver (Y): Drive and Steer towards april tag
    m_driverController.y().whileTrue(new LimelightAlign(drivetrain, "april", 
    () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftX())), // Axes are flipped here on purpose
    () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftY())),true));

    // Driver (X): Just steer towards april tag
    m_driverController.x().whileTrue(new LimelightAlign(drivetrain, "april", 
    () -> xLimiter.calculate(modifyAxis(m_driverController.getLeftX())), // Axes are flipped here on purpose
    () -> yLimiter.calculate(modifyAxis(m_driverController.getLeftY())),false)); */



    // Driver (POV Down): reverse shooter wheels:
    m_driverController1.povDown().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, -0.05, -0.05));
    // Driver (POV Right): reverse shooter wheels:
    m_driverController1.povRight().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, -0.2, -0.2));

    // Driver (POV Up): rereverse shooter wheels:
    m_driverController1.povUp().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, 0.05, 0.05));
    
    // Monkey (B): Deploy intake
    m_driverController1.b().whileTrue(new IntakeDeployCommand(intakeSubsystem, 345, 0.7,0.15));

    // Monkey (A): Handoff note to shooter
    m_driverController1.a().whileTrue(new IntakeHandoffCommand(intakeSubsystem, liftLaunchSubsystem, timer));
   
    //Monkey (Left Bumper): Spit out button
    m_driverController1.leftBumper().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, -0.5, -1, -0.5, -0.05));

    // //Monkey (Right Bumper): Shoot at Amp
     //m_driverController1.rightBumper().whileTrue(new ShooterTimedCommand(liftLaunchSubsystem, intakeSubsystem, 95, 0.009, 0.5, 310));

    m_driverController1.leftTrigger().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0, 105, 0, 0)).onFalse(new SequentialCommandGroup(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0, 105, 0.4, 0.4).withTimeout(1), new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_STORE_ANGLE, 0, LaunchConstants.LAUNCH_ANGLE, 0, 0).withTimeout(0.5)));
    //m_driverController1.rightTrigger().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0, 95, 0.2, 0.2));

    // //Monkey (Y): Shoot at Speaker
    //m_driverController1.y().whileTrue(new ShooterTimedCommand(liftLaunchSubsystem, intakeSubsystem, 39, 0.8, 0.8, 290));
    m_driverController1.y().whileTrue(new SequentialCommandGroup(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, -0.05, -0.1).withTimeout(0.5), new ShooterTimedCommand(liftLaunchSubsystem, intakeSubsystem, 47, 0.8, 0.8, 290, timer))); //deployAngle 43
    //new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, 0.05, 0.05)
    // Monkey (X): Move Arm to Amp Position - Climb 
    m_driverController1.x().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, 320, 0, 84, 0, 0));

    // Monkey (Back): Moves arm down to climb
    m_driverController1.back().whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(liftLaunchSubsystem::enableBreakMode).withTimeout(0.1),
        new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, 345, 0, 0, 0, 0)
      )
    ).onFalse(new InstantCommand(liftLaunchSubsystem::enableCostMode).withTimeout(0.1));

    //Monkey (POV Left): intake from human player
    m_driverController1.povLeft().whileTrue(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, 310, 0, 97, -0.5, -0.5));

   // Monkey (start): Shoot using Limelight
    //m_driverController1.start().whileTrue(new ShootLimelightCommand(liftLaunchSubsystem, intakeSubsystem));

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
    //NamedCommands.registerCommand("storeIntake", new ControlIntake(intakeSubsystem, IntakePos.Store));
     //NamedCommands.registerCommand("intake",  Commands.runOnce(intakeSubsystem::AutoIntake, intakeSubsystem));
    /*NamedCommands.registerCommand("shoot", Commands.runOnce(liftLaunchSubsystem::setShoot, liftLaunchSubsystem));
    NamedCommands.registerCommand("stopshoot", Commands.runOnce(liftLaunchSubsystem::setStore, liftLaunchSubsystem));
    NamedCommands.registerCommand("rotaterobot", Commands.runOnce(drivetrain::rotate180, drivetrain));*/

    NamedCommands.registerCommand("rotateToSpeakerSourceSide", new InstantCommand(() -> drivetrain.setPIDRotateValue(30)));
    NamedCommands.registerCommand("rotateToSpeakerAmpSide", new InstantCommand(() -> drivetrain.setPIDRotateValue(-60)));

    // Intake Commands
    NamedCommands.registerCommand("storeIntake", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_STORE_ANGLE, 0, -1, 0, 0).withTimeout(0.5));
    NamedCommands.registerCommand("intake", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0.3, -1, 0, 0));
    
    // Auto Drivesubsytem Commands
    NamedCommands.registerCommand("holdFast", Commands.runOnce(drivetrain::holdFast, drivetrain));
    NamedCommands.registerCommand("gyroset", Commands.runOnce(drivetrain::zeroGyroscope180, drivetrain));
    NamedCommands.registerCommand("rotateAmp", new InstantCommand(drivetrain::setAmpSideAngle));
    NamedCommands.registerCommand("rotateSource", new InstantCommand(drivetrain::setSourceSideAngle));
    // NamedCommands.registerCommand("trackNote", Commands.runOnce(drivetrain::trackNote, drivetrain));
    // NamedCommands.registerCommand("dontTrackNote", Commands.runOnce(drivetrain::dontTrackNote, drivetrain));
    NamedCommands.registerCommand("usePID", Commands.runOnce(drivetrain::usePathPlanner));
    NamedCommands.registerCommand("noPID", Commands.runOnce(drivetrain::noPathPlanner));
    
    // Auto Shoot Commands
    NamedCommands.registerCommand("firstShoot", new SequentialCommandGroup(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, 290, 0, 39, 0, 0.8).withTimeout(0.2), new ShooterTimedCommandAuto(liftLaunchSubsystem, intakeSubsystem, 39, 0.8, 0.8, 290, timer).withTimeout(1.4)));
    NamedCommands.registerCommand("holdShootPosition", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, 39, 0, 0).withTimeout(0.05));
    NamedCommands.registerCommand("stopshoot", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_STORE_ANGLE, 0, LaunchConstants.LAUNCH_ANGLE, 0, 0).withTimeout(0.5));
    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, -0.05, -0.05).withTimeout(0.5), new ShooterTimedCommandAuto(liftLaunchSubsystem, intakeSubsystem, 39, 0.8, 0.8, 290, timer).withTimeout(1.4)));
    NamedCommands.registerCommand("longShoot", new SequentialCommandGroup(new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, -1, 0, -1, -0.05, -0.05).withTimeout(0.5), new ShooterTimedCommandAuto(liftLaunchSubsystem, intakeSubsystem, 39, 0.9, 0.9, 290, timer).withTimeout(1.4)));

    // Auto Intake Commands // 1.28 - 1.65 - 2.05 - 1.45 - 2
    NamedCommands.registerCommand("IntakeDeploy", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0.3, -1, 0, 0).withTimeout(1.45));
    NamedCommands.registerCommand("IntakeHandoff", new IntakeHandoffCommandAuto(intakeSubsystem, liftLaunchSubsystem, timer).withTimeout(1.65));
    NamedCommands.registerCommand("IntakeDeploySlow", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0.3, -1, 0, 0).withTimeout(2.05));
    NamedCommands.registerCommand("IntakeDeployAmp", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0.3, -1, 0, 0).withTimeout(1.7));
    NamedCommands.registerCommand("IntakeDeploySource", new ManualAngleAndSpeed(liftLaunchSubsystem, intakeSubsystem, IntakeConstants.INTAKE_ANGLE, 0.3, -1, 0, 0).withTimeout(2.5));

    // NamedCommands.registerCommand("EndShoot", new ShooterTimedCommand(liftLaunchSubsystem, intakeSubsystem, 39, 0.8, 0.8, 290, timer));




    NamedCommands.registerCommand("FindNote", 
      new LimelightAlign(drivetrain, "limelight", 
      () -> 0,
      () -> 0, 
      true).withTimeout(2)
    );

    NamedCommands.registerCommand("AlignSpeaker", 
      new LimelightAlign(drivetrain, "april", 
      () -> 0, // Axes are flipped here on purpose
      () -> 0,
      false).withTimeout(2)
    );

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    /* "Example Auto" is file to be deployed on the RoboRIO, This is created by using the PathPlanner Software */
    //SmartDashboard.putData("New New Auto", new PathPlannerAuto("New New Auto"));
    //SmartDashboard.putData("Auto Test", new PathPlannerAuto("Grab and Launch 1"));

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
     return autoChooser.getSelected();//return new PathPlannerAuto("Grab and Launch 1");
  }
}
