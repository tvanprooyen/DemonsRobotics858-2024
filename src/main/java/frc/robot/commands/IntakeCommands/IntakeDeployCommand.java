// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

// Controls the Intake Angle and Motor Speed of Intake
public class IntakeDeployCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double deployAngle, motorSpeedTop, motorSpeedBottom;

  public IntakeDeployCommand( IntakeSubsystem intakeSubsystem, double deployAngle, double motorSpeedTop, double motorSpeedBottom) {
    this.intakeSubsystem = intakeSubsystem;

    //If the angle is -1 then we do not want to control the angle of the intake
    this.deployAngle = deployAngle == -1 ? intakeSubsystem.getDeploySet() : deployAngle;
    this.motorSpeedTop = motorSpeedTop;
    this.motorSpeedBottom = motorSpeedBottom;

    addRequirements(intakeSubsystem);
  }

  



  @Override
  public void initialize() {
    // Set Motors to inputted variables
    intakeSubsystem.setDeploySet(deployAngle);
    intakeSubsystem.setIntakeSpeedBottom(motorSpeedBottom);
    intakeSubsystem.setIntakeSpeedTop(motorSpeedTop);
  }

  @Override
  public void execute() {}

  // Reset motors once command ends
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setDeploySet(IntakeConstants.INTAKE_STORE_ANGLE);
    intakeSubsystem.setIntakeSpeed(0);
  }

  // End Command once the IR Sensor is true
  @Override
  public boolean isFinished() {
    return !intakeSubsystem.getLimitSwitch();
      // return false;
  }
}
