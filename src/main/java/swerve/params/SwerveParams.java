package swerve.params;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import swerve.SDSModuleConfiguration;

public class SwerveParams {
        RobotMaxKinematics maxKinematics;
        SwerveDriveKinematics kinematics;
        SDSModuleConfiguration moduleConfiguration;
        SlotConfigs driveSlotConfigs;
        SpeedLimiterProportions speedLimiterProps;
        PoseEstimationStandardDeviations modulePoseEstStdDevs, visionPoseEstStdDevs;
        SwerveNeutralModes neutralModes;
        SwerveModuleCurrentLimitConfigs swerveCurrentLimitConfigs;
        DriveRampingConfigs driveRampingConfigs;
        CANcoderConfiguration swerveCanCoderConfig;
        HolonomicPathFollowerConfig holonomicPathFollowerConfig;

        TalonFXConfiguration swerveDriveFXConfig, swerveSteerFXconfig;

        public SwerveParams(RobotMaxKinematics maxKinematics, SwerveDriveKinematics kinematics,
                        SDSModuleConfiguration moduleConfiguration,
                        HolonomicPathFollowerConfig holonomicPathFollowerConfig, SlotConfigs driveSlotConfigs,
                        PoseEstimationStandardDeviations modulePoseEstStdDevs,
                        PoseEstimationStandardDeviations visionPoseEstStdDevs) {
                this(maxKinematics, kinematics, moduleConfiguration, holonomicPathFollowerConfig,
                                new SwerveModuleCurrentLimitConfigs(), driveSlotConfigs, modulePoseEstStdDevs,
                                visionPoseEstStdDevs, new DriveRampingConfigs(),
                                new SpeedLimiterProportions(), new SwerveNeutralModes());
        }

        public SwerveParams(RobotMaxKinematics maxKinematics,
                        SwerveDriveKinematics kinematics,
                        SDSModuleConfiguration moduleConfiguration,
                        HolonomicPathFollowerConfig holonomicPathFollowerConfig,
                        SwerveModuleCurrentLimitConfigs currentLimitConfigs,
                        SlotConfigs driveSlotConfigs,
                        PoseEstimationStandardDeviations modulePoseEstStdDevs,
                        PoseEstimationStandardDeviations visionPoseEstStdDevs,
                        DriveRampingConfigs driveRampingConfigs,
                        SpeedLimiterProportions speedLimiterProportions,
                        SwerveNeutralModes neutralModes) {
                this.maxKinematics = maxKinematics;
                this.kinematics = kinematics;
                this.moduleConfiguration = moduleConfiguration;
                this.holonomicPathFollowerConfig = holonomicPathFollowerConfig;
                this.swerveCurrentLimitConfigs = currentLimitConfigs;
                this.driveSlotConfigs = driveSlotConfigs;
                this.modulePoseEstStdDevs = modulePoseEstStdDevs;
                this.visionPoseEstStdDevs = visionPoseEstStdDevs;
                this.driveRampingConfigs = driveRampingConfigs;
                this.speedLimiterProps = speedLimiterProportions;
                this.neutralModes = neutralModes;

                setSwerveConfigs();
        }

        /**
         * Must be called AFTER values have been set
         */
        void setSwerveConfigs() {
                /*
                 * Drive Motor Configuration
                 */
                swerveDriveFXConfig = new TalonFXConfiguration();

                var motorOutputConfigs = new MotorOutputConfigs();

                motorOutputConfigs.withInverted(moduleConfiguration.driveMotorInvert)
                                .withNeutralMode(neutralModes.drive);

                swerveDriveFXConfig.withMotorOutput(motorOutputConfigs);
                swerveDriveFXConfig.withCurrentLimits(swerveCurrentLimitConfigs.drive);
                swerveDriveFXConfig.withSlot0(Slot0Configs.from(driveSlotConfigs));
                swerveDriveFXConfig.withOpenLoopRamps(driveRampingConfigs.openLoopConfigs);
                swerveDriveFXConfig.withClosedLoopRamps(driveRampingConfigs.closedLoopConfigs);

                /*
                 * Steer Motor Configuration
                 */
                swerveSteerFXconfig = new TalonFXConfiguration();

                motorOutputConfigs = new MotorOutputConfigs();

                motorOutputConfigs.withInverted(moduleConfiguration.angleMotorInvert)
                                .withNeutralMode(neutralModes.steer);

                swerveSteerFXconfig.withCurrentLimits(swerveCurrentLimitConfigs.steer);
                swerveSteerFXconfig.withSlot0(Slot0Configs.from(moduleConfiguration.angleSlotConfigs));

                /*
                 * CANCoder Configuration
                 */
                swerveCanCoderConfig = new CANcoderConfiguration();

                var magnetSensorConfig = new MagnetSensorConfigs();

                magnetSensorConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                                .withSensorDirection(moduleConfiguration.canCoderSensorDirection);

                swerveCanCoderConfig.withMagnetSensor(magnetSensorConfig);
        }

        public RobotMaxKinematics getMaxKinematics() {
                return this.maxKinematics;
        }

        public SwerveDriveKinematics getKinematics() {
                return this.kinematics;
        }

        public SDSModuleConfiguration getModuleConfiguration() {
                return this.moduleConfiguration;
        }

        public SlotConfigs getDriveSlotConfigs() {
                return this.driveSlotConfigs;
        }

        public SpeedLimiterProportions getSpeedLimiterProps() {
                return this.speedLimiterProps;
        }

        public PoseEstimationStandardDeviations getModulePoseEstStdDevs() {
                return this.modulePoseEstStdDevs;
        }

        public PoseEstimationStandardDeviations getVisionPoseEstStdDevs() {
                return this.visionPoseEstStdDevs;
        }

        public SwerveNeutralModes getNeutralModes() {
                return this.neutralModes;
        }

        public SwerveModuleCurrentLimitConfigs getSwerveCurrentLimitConfigs() {
                return this.swerveCurrentLimitConfigs;
        }

        public DriveRampingConfigs getDriveRampingConfigs() {
                return this.driveRampingConfigs;
        }

        public CANcoderConfiguration getSwerveCanCoderConfig() {
                return this.swerveCanCoderConfig;
        }

        public HolonomicPathFollowerConfig getHolonomicPathFollowerConfig() {
                return this.holonomicPathFollowerConfig;
        }

        public TalonFXConfiguration getDriveFxConfiguration() {
                return this.swerveDriveFXConfig;
        }

        public TalonFXConfiguration getSteeFxConfiguration() {
                return this.swerveSteerFXconfig;
        }

        public static SimpleMotorFeedforward SlotConfigs2SimpleMotorFeedForward(SlotConfigs configs) {
                return new SimpleMotorFeedforward(configs.kS, configs.kV, configs.kA);
        }

        public ProfiledPIDController getSteerPidController() {
                return new ProfiledPIDController(moduleConfiguration.angleSlotConfigs.kP, 0,
                                0,
                                new TrapezoidProfile.Constraints(0, 0));
        }
}
