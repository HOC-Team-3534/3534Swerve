package swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class SDSModuleConfiguration {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKS;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue canCoderSensorDirection;

    public SDSModuleConfiguration(double wheelDiameter, double angleGearRatio,
            double driveGearRatio, double angleKP,
            double angleKI, double angleKD,
            double angleKS, InvertedValue driveMotorInvert,
            InvertedValue angleMotorInvert,
            SensorDirectionValue canCoderSensorDirection) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKS = angleKS;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.canCoderSensorDirection = canCoderSensorDirection;
    }

    /** Swerve Drive Specialties - MK3 Module */
    public static SDSModuleConfiguration SDSMK3(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);
        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);
        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKS = 0.0;
        InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        SensorDirectionValue canCoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        return new SDSModuleConfiguration(wheelDiameter, angleGearRatio,
                driveGearRatio, angleKP, angleKI,
                angleKD, angleKS, driveMotorInvert,
                angleMotorInvert, canCoderSensorDirection);
    }

    /** Swerve Drive Specialties - MK4 Module */
    public static SDSModuleConfiguration SDSMK4(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);
        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);
        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKS = 0.0;
        InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        SensorDirectionValue canCoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        return new SDSModuleConfiguration(wheelDiameter, angleGearRatio,
                driveGearRatio, angleKP, angleKI,
                angleKD, angleKS, driveMotorInvert,
                angleMotorInvert, canCoderSensorDirection);
    }

    /** Swerve Drive Specialties - MK4i Module */
    public static SDSModuleConfiguration SDSMK4i(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);
        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);
        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKS = 0.0;
        InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        SensorDirectionValue canCoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        return new SDSModuleConfiguration(wheelDiameter, angleGearRatio,
                driveGearRatio, angleKP, angleKI,
                angleKD, angleKS, driveMotorInvert,
                angleMotorInvert, canCoderSensorDirection);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios {
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_Standard = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_Fast = (6.86 / 1.0);
        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75 / 1.0);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12 / 1.0);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14 / 1.0);
        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
    }
}