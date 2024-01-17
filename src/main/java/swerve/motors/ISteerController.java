package swerve.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.params.SwerveParams;

public interface ISteerController {
    void config(SwerveParams swerveParams);

    void resetToAbsolute();

    void setAngle(SwerveModuleState desiredState);

    double getVoltage();

    Rotation2d getRate();

    Rotation2d getAngle();

    void setVoltage(double voltage);
}