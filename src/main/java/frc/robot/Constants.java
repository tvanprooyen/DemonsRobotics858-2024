// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort1 = 1;
  }

  /**
   * This is where we setup all the constants for our swerve modules
   */
  public static final class SDSConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794; //0.635mm == 25"
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;

    public static final int DRIVETRAIN_PIGEON_ID = 5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(335); //335155 + 40 - 15

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(50+98); //50+98 + 5 + 40 - 10

    //BL Encoder LED Burnt Out
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(0); // + 26 - 10

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 9;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(5); //+3+90 + 32 - 10
  }
  /**
   * Anything PathPlanner needs for Constraints
   */
  public static final class PathPlannerConstants {
    //Swerve Drive (Holonomic) Pathplanner Configuration
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(4.0, 0, 0), // Translation constants 
      new PIDConstants(1, 0.00, 0.00), // Rotation constants  //old 1
      SDSConstants.MAX_VELOCITY_METERS_PER_SECOND, 
      // new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), // Drive base radius (distance from center to furthest module) 
      new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }

  public static final class IntakeConstants {
    // Angle Constants
    public final static double INTAKE_HANDOFF_ANGLE = 230; //240
    public final static double INTAKE_STORE_ANGLE = 285; //275
    public final static double AUTO_INTAKE_STORE_ANGLE = 310;
    public final static double INTAKE_ANGLE = 353; //345
  }

  public static final class LaunchConstants {
    // Angle Constants
    public final static double LAUNCH_ANGLE = 10;
    public final static double SHOOTER_HANDOFF_ANGLE = 15;
  }

  public static final class LimelightConstants {
      // Limits of Height to Target: Low: 1.985, Middle: 2.045, Max: 2.11
      // Limits of Distance from Speaker: Low: 0, Middle: 0.457, Max: 0.457
      public final static double HEIGHT_TO_TARGET = 1.985; 
      public final static double LENGTH_FROM_LIMELIGHT_TO_SHOOTER_AXIS = 0.584; //shouldn't change unless limelight moves
      public final static double DISTANCE_FROM_SPEAKER = 0.2; 
  }
}
