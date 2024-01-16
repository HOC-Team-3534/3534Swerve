package swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
        public static RobotMaxKinematics maxKinematics;
        public static SwerveDriveKinematics kinematics;
        public static SDSModuleConfiguration moduleConfiguration;
        public static SlotConfigs driveSlotConfigs;
        public static SpeedLimiterProportions speedLimiterProps;
        public static PoseEstimationStandardDeviations modulePoseEstStdDevs, visionPoseEstStdDevs;
        public static SwerveNeutralModes neutralModes;
        public static SwerveModuleCurrentLimitConfigs swerveCurrentLimitConfigs;
        public static DriveRampingConfigs driveRampingConfigs;
        public static CANcoderConfiguration swerveCanCoderConfig;
        public static HolonomicPathFollowerConfig holonomicPathFollowerConfig;

        public static TalonFXConfiguration swerveDriveFXConfig, swerveAngleFXconfig;

        public static void fillNecessaryConstantsForFalcon(RobotMaxKinematics maxKinematics_,
                        SwerveDriveKinematics kinematics_,
                        SDSModuleConfiguration moduleConfiguration_,
                        HolonomicPathFollowerConfig holonomicPathFollowerConfig_,
                        SwerveModuleCurrentLimitConfigs currentLimitConfigs_,
                        SlotConfigs driveSlotConfigs_,
                        DriveRampingConfigs driveRampingConfigs_,
                        SpeedLimiterProportions speedLimiterProportions_,
                        SwerveNeutralModes neutralModes_) {
                maxKinematics = maxKinematics_;
                kinematics = kinematics_;
                moduleConfiguration = moduleConfiguration_;
                holonomicPathFollowerConfig = holonomicPathFollowerConfig_;
                swerveCurrentLimitConfigs = currentLimitConfigs_;
                driveSlotConfigs = driveSlotConfigs_;
                driveRampingConfigs = driveRampingConfigs_;
                speedLimiterProps = speedLimiterProportions_;
                neutralModes = neutralModes_;
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

                motorOutputConfigs.withInverted(SwerveConstants.moduleConfiguration.driveMotorInvert)
                                .withNeutralMode(SwerveConstants.neutralModes.drive);

                swerveDriveFXConfig.withMotorOutput(motorOutputConfigs);
                swerveDriveFXConfig.withCurrentLimits(swerveCurrentLimitConfigs.drive);
                swerveDriveFXConfig.withSlot0(Slot0Configs.from(driveSlotConfigs));
                swerveDriveFXConfig.withOpenLoopRamps(driveRampingConfigs.openLoopConfigs);
                swerveDriveFXConfig.withClosedLoopRamps(driveRampingConfigs.closedLoopConfigs);

                /*
                 * Steer Motor Configuration
                 */
                swerveAngleFXconfig = new TalonFXConfiguration();

                motorOutputConfigs = new MotorOutputConfigs();

                motorOutputConfigs.withInverted(moduleConfiguration.angleMotorInvert)
                                .withNeutralMode(neutralModes.steer);

                swerveAngleFXconfig.withCurrentLimits(swerveCurrentLimitConfigs.steer);
                swerveAngleFXconfig.withSlot0(Slot0Configs.from(moduleConfiguration.angleSlotConfigs));

                /*
                 * CANCoder Configuration
                 */
                swerveCanCoderConfig = new CANcoderConfiguration();

                var magnetSensorConfig = new MagnetSensorConfigs();

                magnetSensorConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                                .withSensorDirection(moduleConfiguration.canCoderSensorDirection);

                swerveCanCoderConfig.withMagnetSensor(magnetSensorConfig);
        }

        public class SpeedLimiterProportions {
                public double fastDriveProp, fastSteerProp, slowDriveProp, slowSteerProp;

                public SpeedLimiterProportions() {
                        this.fastDriveProp = 1;
                        this.fastSteerProp = 1;
                        this.slowDriveProp = 0.25;
                        this.slowSteerProp = 0.25;
                }

                public SpeedLimiterProportions(double fastDriveProp, double fastSteerProp, double slowDriveProp,
                                double slowSteerProp) {
                        this.fastDriveProp = fastDriveProp;
                        this.fastSteerProp = fastSteerProp;
                        this.slowDriveProp = slowDriveProp;
                        this.slowSteerProp = slowSteerProp;
                }
        }

        public class PoseEstimationStandardDeviations {
                public double x, y;
                public Rotation2d angle;

                public PoseEstimationStandardDeviations() {
                        this.x = 0.01;
                        this.y = 0.01;
                        this.angle = Rotation2d.fromDegrees(2);
                }

                public PoseEstimationStandardDeviations(double x, double y, Rotation2d angle) {
                        this.x = x;
                        this.y = y;
                        this.angle = angle;
                }
        }

        public class SwerveModuleCurrentLimitConfigs {
                public CurrentLimitsConfigs drive, steer;

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

        public class DriveRampingConfigs {
                public OpenLoopRampsConfigs openLoopConfigs;
                public ClosedLoopRampsConfigs closedLoopConfigs;

                public DriveRampingConfigs() {
                        this.openLoopConfigs = (new OpenLoopRampsConfigs()).withDutyCycleOpenLoopRampPeriod(0.25);
                        this.closedLoopConfigs = new ClosedLoopRampsConfigs();
                }

                public DriveRampingConfigs(OpenLoopRampsConfigs openLoopRampsConfigs,
                                ClosedLoopRampsConfigs closedLoopRampsConfigs) {
                        this.openLoopConfigs = openLoopRampsConfigs;
                        this.closedLoopConfigs = closedLoopRampsConfigs;
                }
        }

        public class RobotMaxKinematics {
                public double vel, angVel, angAccel;
        }

        public class SwerveNeutralModes {
                public NeutralModeValue drive, steer;

                public SwerveNeutralModes() {
                        drive = NeutralModeValue.Brake;
                        steer = NeutralModeValue.Coast;
                }
        }

        public static SimpleMotorFeedforward SlotConfigs2SimpleMotorFeedForward(SlotConfigs configs) {
                return new SimpleMotorFeedforward(configs.kS, configs.kV, configs.kA);
        }

        public static ProfiledPIDController getSteerPidController() {
                return new ProfiledPIDController(SwerveConstants.moduleConfiguration.angleSlotConfigs.kP, 0,
                                0,
                                new TrapezoidProfile.Constraints(0, 0));
        }
}
