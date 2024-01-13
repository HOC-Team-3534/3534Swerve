package swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {
    /**
     * Minimize the change in heading the desired swerve module state would
     * require by potentially reversing the direction the wheel spins.
     * Customized from WPILib's version to include placing in appropriate scope
     * for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState,
            Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180)
                    : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed,
                Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * 
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        scopeReference %= 360;
        newAngle %= 360;
    
        double delta = newAngle - scopeReference;
    
        // Adjust delta to be within -180 to 180 degrees
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
    
        return scopeReference + delta;
    }
    
}
