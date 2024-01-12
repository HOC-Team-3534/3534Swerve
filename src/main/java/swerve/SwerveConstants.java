package swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    public static double maxSpeed;
    public static double moduleMaxAngularVel, moduleMaxAngularAccel;
    public static double robotMaxAngularVel, robotMaxAngularAccel;
    public static SwerveDriveKinematics kinematics;
    public static SDSModuleConfiguration moduleConfiguration;
    public static double driveKP, driveKS, driveKV, driveKA;
    // only need to set steer values if using basic swerve module
    public static double steerKP, steerKS, steerKV, steerKA, steerKI, steerKD,
            steerKF;
    public static double autonDriveKP, autonSteerKP;
    public static double fastDriveProp, fastSteerProp, slowDriveProp,
            slowSteerProp;
    public static double modulePoseEstXStdDev = 0.01,
            modulePoseEstYStdDev = 0.01, visionPoseEstXStdDev = 0.01,
            visionPoseEstYStdDev = 0.01;
    public static Rotation2d modulePoseEstAngleStdDev = Rotation2d.fromDegrees(0.5);
    public static Rotation2d visionPoseEstAngleStdDev = Rotation2d.fromDegrees(5);
    public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake,
            angleNeutralMode = NeutralModeValue.Coast;
    public static IdleMode driveIdleMode = (driveNeutralMode == NeutralModeValue.Brake) ? IdleMode.kBrake
            : IdleMode.kCoast,
            angleIdleMode = (angleNeutralMode == NeutralModeValue.Brake) ? IdleMode.kBrake
                    : IdleMode.kCoast;
    public static boolean angleEnableCurrentLimit = true,
            driveEnableCurrentLimit = true;
    public static int angleContinuousCurrentLimit = 25,
            anglePeakCurrentLimit = 40,
            driveContinuousCurrentLimit = 35,
            drivePeakCurrentLimit = 60;
    public static double anglePeakCurrentDuration = 0.1,
            drivePeakCurrentDuration = 0.1;
    public static double openLoopRamp = 0.25, closedLoopRamp = 0.0;
    public static AbsoluteSensorRangeValue absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    public static TalonFXConfiguration swerveDriveFXConfig, swerveAngleFXconfig;
    public static CANcoderConfiguration swerveCanCoderConfig;

    public static void fillNecessaryConstantsForFalcon(double maxSpeed_,
            double robotMaxAngularVel_,
            double robotMaxAngularAccel_,
            SwerveDriveKinematics kinematics_,
            SDSModuleConfiguration moduleConfiguration_,
            double driveKP_,
            double driveKS_,
            double driveKV_,
            double driveKA_,
            double autonDriveKP_,
            double autonSteerKP_,
            double fastDriveProp_,
            double fastSteerProp_,
            double slowDriveProp_,
            double slowSteerProp_) {
        maxSpeed = maxSpeed_;
        robotMaxAngularVel = robotMaxAngularVel_;
        robotMaxAngularAccel = robotMaxAngularAccel_;
        kinematics = kinematics_;
        moduleConfiguration = moduleConfiguration_;
        driveKP = driveKP_;
        driveKS = driveKS_;
        driveKV = driveKV_;
        driveKA = driveKA_;
        autonDriveKP = autonDriveKP_;
        autonSteerKP = autonSteerKP_;
        fastDriveProp = fastDriveProp_;
        fastSteerProp = fastSteerProp_;
        slowDriveProp = slowDriveProp_;
        slowSteerProp = slowSteerProp_;
    }

    /**
     * Must be called AFTER values have been set
     */
    public static void createSwerveConstants() {
        /*
         * Drive Motor Configuration
         */
        swerveDriveFXConfig = new TalonFXConfiguration();

        var motorOutputConfigs = new MotorOutputConfigs();
        var currentLimitsConfig = new CurrentLimitsConfigs();
        var slot0Configs = new Slot0Configs();
        var openLoopRampsConfigs = new OpenLoopRampsConfigs();
        var closedLoopRampConfigs = new ClosedLoopRampsConfigs();

        motorOutputConfigs.withInverted(SwerveConstants.moduleConfiguration.driveMotorInvert).withNeutralMode(SwerveConstants.driveNeutralMode);
        currentLimitsConfig.withSupplyCurrentLimitEnable(driveEnableCurrentLimit).withSupplyCurrentLimit(driveContinuousCurrentLimit).withSupplyCurrentThreshold(drivePeakCurrentLimit).withSupplyTimeThreshold(drivePeakCurrentDuration);
        slot0Configs.withKP(driveKP);
        openLoopRampsConfigs.withDutyCycleOpenLoopRampPeriod(openLoopRamp); // TODO check openLoopRamp and closedLoopRamp. Is time? or voltage?
        closedLoopRampConfigs.withDutyCycleClosedLoopRampPeriod(closedLoopRamp);

        swerveDriveFXConfig.withMotorOutput(motorOutputConfigs);
        swerveDriveFXConfig.withCurrentLimits(currentLimitsConfig);
        swerveDriveFXConfig.withSlot0(slot0Configs);
        swerveDriveFXConfig.withOpenLoopRamps(openLoopRampsConfigs);
        swerveDriveFXConfig.withClosedLoopRamps(closedLoopRampConfigs);

        /*
         * Steer Motor Configuration
         */
        swerveAngleFXconfig = new TalonFXConfiguration();

        motorOutputConfigs = new MotorOutputConfigs();
        currentLimitsConfig = new CurrentLimitsConfigs();
        slot0Configs = new Slot0Configs();

        motorOutputConfigs.withInverted(moduleConfiguration.angleMotorInvert).withNeutralMode(angleNeutralMode);
        currentLimitsConfig.withSupplyCurrentLimitEnable(angleEnableCurrentLimit).withSupplyCurrentLimit(angleContinuousCurrentLimit).withSupplyCurrentThreshold(anglePeakCurrentLimit).withSupplyTimeThreshold(anglePeakCurrentDuration);
        slot0Configs.withKP(moduleConfiguration.angleKP).withKI(moduleConfiguration.angleKI).withKD(moduleConfiguration.angleKD).withKS(moduleConfiguration.angleKS);

        swerveAngleFXconfig.withCurrentLimits(currentLimitsConfig);
        swerveAngleFXconfig.withSlot0(slot0Configs);

        /*
         * CANCoder Configuration
         */
        swerveCanCoderConfig = new CANcoderConfiguration();

        var magnetSensorConfig = new MagnetSensorConfigs();

        magnetSensorConfig.withAbsoluteSensorRange(absoluteSensorRange).withSensorDirection(moduleConfiguration.canCoderSensorDirection);

        swerveCanCoderConfig.withMagnetSensor(magnetSensorConfig);
    }
}
