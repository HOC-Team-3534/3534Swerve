package swerve.motors;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface IDriveController {
    void config();

    double getVoltage();

    /**
     * @return the velocity in meters per second
     */
    double getVelocity();

    /**
     * @return the distance in meters
     */
    double getDistance();

    void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * @param voltage the voltage to send to the drive motor
     */
    void setVoltage(double voltage);
}
