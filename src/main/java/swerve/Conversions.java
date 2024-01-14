package swerve;

public class Conversions {
    /**
     * @param degrees   Degrees of Rotation of Mechanism
     * @param gearRatio Gear Ratio Between Falcon and Mechanism
     * 
     * @return Falcon Rotations
     */
    public static double steerDegreesToFalconRotations(double degrees, double gearRatio) {
        return degrees / 360.0 / gearRatio;
    }

    /**
     * @param velocityRPS Falcon Velocity Rotations per Second
     * @param gearRatio   Gear Ratio Between Falcon and Mechanism
     * 
     * @return Rotations of Mechanism per Second
     */
    public static double falconRotationsToWheelRotations(double velocityRPS,
            double gearRatio) {
        return velocityRPS / gearRatio;
    }

    /**
     * @param wheelRotations Wheel Rotations
     * @param gearRatio      Gear Ratio Between Falcon and Mechanism
     * 
     * @return Rotations of Falcon per Second
     */

    public static double wheelRotationsToFalconRotations(double wheelRotations, double gearRatio) {
        return wheelRotations * gearRatio;
    }

    /**
     * @param velocityRPS   Falcon Velocity Rotations per Second
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism
     * 
     * @return Falcon Velocity Counts
     */
    public static double falconRotationsPerSecondToWheelMetersPerSecond(double velocityRPS,
            double circumference, double gearRatio) {
        return falconRotationsToWheelRotations(velocityRPS, gearRatio) * circumference;

    }

    /**
     * @param positionRotations Falcon Position Rotations
     * @param circumference     Circumference of Wheel
     * @param gearRatio         Gear Ratio Between Falcon and Mechanism
     * 
     * @return Meters
     */
    public static double falconRotationsToWheelMeters(double positionRotations,
            double circumference,
            double gearRatio) {
        return positionRotations / gearRatio * circumference;
    }

    /**
     * @param wheelMPS      Wheel Meters per Second
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio Between Falcon and Mechanism
     * 
     * @return Falcons Rotations per Second
     */

    public static double wheelMetersPerSecondToFalconRotationsPerSecond(double wheelMPS, double circumference,
            double gearRatio) {
        return wheelRotationsToFalconRotations(wheelMPS, gearRatio) / circumference;
    }
}