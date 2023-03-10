package swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveDrivetrainModel {
    public final static int NUM_MODULES = 4;
    final SwerveModule[] modules = new SwerveModule[NUM_MODULES];
    final Gyro gyro;
    final SwerveDrivePoseEstimator poseEstimator;
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();
    final HolonomicDriveController holo;
    final ProfiledPIDController thetaController;
    Rotation2d simGyroAngleCache = new Rotation2d();

    public SwerveDrivetrainModel(SwerveModule frontLeftModule,
            SwerveModule frontRightModule,
            SwerveModule backLeftModule,
            SwerveModule backRighModule, Gyro gyro) {
        modules[0] = frontLeftModule;
        modules[1] = frontRightModule;
        modules[2] = backLeftModule;
        modules[3] = backRighModule;
        this.gyro = gyro;
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
        thetaController = new ProfiledPIDController(SwerveConstants.autonSteerKP,
                0, 0,
                new TrapezoidProfile.Constraints(SwerveConstants.robotMaxAngularVel,
                        SwerveConstants.robotMaxAngularAccel));
        holo = new HolonomicDriveController(new PIDController(SwerveConstants.autonDriveKP,
                0, 0),
                new PIDController(SwerveConstants.autonDriveKP,
                        0, 0),
                thetaController);
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        poseEstimator.update(getRawGyroHeading(), getModulePositions());
    }

    public void updateOdometryWithVision(Pose2d botPose, double latency) {
        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - latency); // TODO determine latency of
                                                                                         // camera based on timestamps
                                                                                         // or other
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

    public void setModuleStates(SwerveInput input, Rotation2d desiredRotation,
            boolean creep, boolean resetController) {
        var driveProp = creep ? SwerveConstants.slowDriveProp
                : SwerveConstants.fastDriveProp;
        var modMaxSpeed = driveProp * SwerveConstants.maxSpeed;
        input = handleStationary(input);
        if (resetController)
            thetaController.reset(getGyroHeading().getRadians());
        var angularSpeed = thetaController.calculate(getGyroHeading().getRadians(),
                desiredRotation.getRadians());
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(input.m_translationX * modMaxSpeed,
                input.m_translationY * modMaxSpeed, angularSpeed, getGyroHeading()), false);
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
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Command createOnTheFlyPathCommand(Pose2d endPose,
            Rotation2d endHeading, double endVelocity, double autonMaxSpeed,
            double autonMaxAccel, SwerveSubsystem m_drive) {
        var xy_vel = new Translation2d(getSpeeds().vxMetersPerSecond,
                getSpeeds().vyMetersPerSecond).getNorm();
        var rot_vel = Math.abs(getSpeeds().omegaRadiansPerSecond);
        var stillHeading = endPose.getTranslation().minus(getPose().getTranslation()).getAngle();
        PathPoint startPoint = (xy_vel <= 0.05 && rot_vel <= 0.02)
                ? new PathPoint(getPose().getTranslation(), stillHeading,
                        getPose().getRotation())
                : PathPoint.fromCurrentHolonomicState(getPose(), getSpeeds());
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(autonMaxSpeed,
                autonMaxAccel), startPoint,
                new PathPoint(endPose.getTranslation(),
                        endHeading,
                        endPose.getRotation(),
                        endVelocity));
        return createCommandForTrajectory(trajectory, m_drive, false, false);
    }

    private void resetHoloController() {
        holo.getXController().reset();
        holo.getYController().reset();
        holo.getThetaController().reset(this.getGyroHeading().getRadians());
    }

    public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, SwerveSubsystem m_drive,
            boolean resetToInitial, boolean useAlliance) {
        if (resetToInitial)
            setKnownPose(trajectory.getInitialHolonomicPose());
        MyPPSwerveControllerCommand swerveControllerCommand = new MyPPSwerveControllerCommand(
                trajectory,
                () -> getPose(), // Functional interface to feed supplier
                SwerveConstants.kinematics,
                holo, () -> resetHoloController(),
                commandStates -> setModuleStates(commandStates, false), useAlliance,
                m_drive);
        return swerveControllerCommand.andThen(() -> setVoltageToZero());
    }

    public void goToPose(PathPlannerState state) {
        setModuleStates(
                holo.calculate(getPose(), state.poseMeters, state.velocityMetersPerSecond, state.holonomicRotation),
                false);
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
