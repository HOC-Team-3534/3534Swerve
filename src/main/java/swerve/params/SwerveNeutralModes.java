package swerve.params;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveNeutralModes {
    public final NeutralModeValue drive, steer;

    public SwerveNeutralModes() {
        drive = NeutralModeValue.Brake;
        steer = NeutralModeValue.Coast;
    }
}
