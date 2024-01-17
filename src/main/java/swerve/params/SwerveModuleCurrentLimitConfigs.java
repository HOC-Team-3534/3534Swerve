package swerve.params;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class SwerveModuleCurrentLimitConfigs {
    public final CurrentLimitsConfigs drive, steer;

    public SwerveModuleCurrentLimitConfigs() {
        this.drive = new CurrentLimitsConfigs();
        this.drive.SupplyCurrentLimitEnable = true;
        this.drive.SupplyCurrentLimit = 35;
        this.drive.SupplyCurrentThreshold = 60;
        this.drive.SupplyTimeThreshold = 0.1;

        this.steer = new CurrentLimitsConfigs();
        this.steer.SupplyCurrentLimitEnable = true;
        this.steer.SupplyCurrentLimit = 25;
        this.steer.SupplyCurrentThreshold = 40;
        this.steer.SupplyTimeThreshold = 0.1;
    }

    public SwerveModuleCurrentLimitConfigs(CurrentLimitsConfigs drive, CurrentLimitsConfigs steer) {
        this.drive = drive;
        this.steer = steer;
    }
}
