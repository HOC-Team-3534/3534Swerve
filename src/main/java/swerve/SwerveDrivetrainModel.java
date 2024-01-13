package swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivetrainModel {
    public final static int NUM_MODULES = 4;
    final SwerveModule[] modules = new SwerveModule[NUM_MODULES];
    final Pigeon2 pigeon;
    final SwerveDrivePoseEstimator poseEstimator;
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();
    Rotation2d simGyroAngleCache = new Rotation2d();

    public SwerveDrivetrainModel(SwerveModule frontLeftModule,
            SwerveModule frontRightModule,
            SwerveModule backLeftModule,
            SwerveModule backRighModule, Pigeon2 pigeon, SwerveSubsystem swerve) {
        modules[0] = frontLeftModule;
        modules[1] = frontRightModule;
        modules[2] = backLeftModule;
        modules[3] = backRighModule;
        this.pigeon = pigeon;

        AutoBuilder.configureHolonomic(this::getPose, this::setKnownPose, this::getSpeeds,
                (speeds) -> this.setModuleStates(speeds, false), SwerveConstants.holonomicPathFollowerConfig,
                () -> false,
                swerve);
        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry
         * readings. The numbers used below are robot specific, and should be
         * tuned.
         */
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics,
                getRawGyroHeading(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(SwerveConstants.modulePoseEstXStdDev, SwerveConstants.modulePoseEstYStdDev,
                        SwerveConstants.modulePoseEstAngleStdDev.getRadians()),
                VecBuilder.fill(SwerveConstants.visionPoseEstXStdDev, SwerveConstants.visionPoseEstYStdDev,
                        SwerveConstants.visionPoseEstAngleStdDev.getRadians()));
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        poseEstimator.update(getRawGyroHeading(), getModulePositions());
    }

    public void updateOdometryWithVision(Pose2d botPose, double timeValue, boolean isTimestamp) {
        if (isTimestamp)
            poseEstimator.addVisionMeasurement(botPose, timeValue);
        else
            poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - timeValue);
    }

    public void setModuleStates(SwerveInput input, boolean creep,
            boolean isOpenLoop) {
        var driveProp = creep ? SwerveConstants.slowDriveProp
                : SwerveConstants.fastDriveProp;
        var steerProp = creep ? SwerveConstants.slowSteerProp
                : SwerveConstants.fastSteerProp;
        var modMaxSpeed = driveProp * SwerveConstants.maxSpeed;
        var modMaxAngularSpeed = steerProp * SwerveConstants.robotMaxAngularVel;
        input = handleStationary(input);
        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(input.m_translationX * modMaxSpeed,
                        input.m_translationY * modMaxSpeed, input.m_rotation * modMaxAngularSpeed, getGyroHeading()),
                        isOpenLoop);
                break;

            case "Robot Oriented":
                setModuleStates(new ChassisSpeeds(input.m_translationX * modMaxSpeed,
                        input.m_translationY * modMaxSpeed,
                        input.m_rotation * modMaxAngularSpeed), isOpenLoop);
                break;
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        if (RobotBase.isSimulation()) {
            var chassisSpeeds = SwerveConstants.kinematics.toChassisSpeeds(states);
            simGyroAngleCache = simGyroAngleCache.plus(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.020));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxSpeed);
        for (int i = 0; i < NUM_MODULES; i++) {
            modules[i].setDesiredState(states[i], isOpenLoop);
        }
    }

    public void setVoltageToZero() {
        for (int i = 0; i < NUM_MODULES; i++) {
            modules[i].setDriveVoltageForCharacterization(0);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.kinematics.toChassisSpeeds(getSwerveModuleStates());

    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setKnownPose(Pose2d in) {
        poseEstimator.resetPosition(getRawGyroHeading(), getModulePositions(), in);
    }

    private Rotation2d getRawGyroHeading() {
        if (RobotBase.isSimulation())
            return simGyroAngleCache;
        return pigeon.getRotation2d();
    }

    public Rotation2d getGyroHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Command pathfindToPose(Pose2d endPose, double endVelocity, double autonMaxSpeed,
            double autonMaxAccel, double autonMaxAngSpeed, double autonMaxAngAccel) {
        return AutoBuilder.pathfindToPose(endPose,
                new PathConstraints(autonMaxSpeed, autonMaxAccel, autonMaxAngSpeed, autonMaxAngAccel), endVelocity);
    }

    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    private SwerveInput handleStationary(SwerveInput input) {
        if (input.m_rotation == 0 && input.m_translationX == 0
                && input.m_translationY == 0) {
            // Hopefully this will turn all of the modules to the "turning" configuration so
            // being pushed is more difficult
            input.m_rotation = 0.0; // 001;
        }
        return input;
    }

    public SwerveModule[] getSwerveModules() {
        return modules;
    }
}
