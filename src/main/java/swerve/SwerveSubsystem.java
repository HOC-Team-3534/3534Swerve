// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivetrainModel dt;

    public SwerveSubsystem(SwerveDrivetrainModel dt) {
        this.dt = dt;
    }

    @Override
    public void periodic() {
        dt.updateOdometry();
    }
}