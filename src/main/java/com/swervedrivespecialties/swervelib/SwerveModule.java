package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getDrivePosistion();

    double getSteerAngle();

    double getSteerPosistion();

    SwerveModulePosition getPosition();

    void stop();

    void set(double driveVoltage, double steerAngle);
}
