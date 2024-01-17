package swerve.motors;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.params.SwerveParams;

public interface IDriveController {
    void config(SwerveParams swerveParams);

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
     * @param voltage
     *            the voltage to send to the drive motor
     */
    void setVoltage(double voltage);
}
