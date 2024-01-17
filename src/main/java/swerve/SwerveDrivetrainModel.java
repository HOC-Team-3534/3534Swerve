package swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import swerve.params.SwerveParams;
import swerve.path.IPathPlanner;

public class SwerveDrivetrainModel {
    public final static int NUM_MODULES = 4;
    final ArrayList<SwerveModule> modules;
    final Pigeon2 pigeon;
    final SwerveDrivePoseEstimator poseEstimator;
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();
    Rotation2d simGyroAngleCache = new Rotation2d();
    final SwerveParams swerveParams;
    IPathPlanner pathPlanner;

    public SwerveDrivetrainModel(SwerveModule frontLeftModule,
            SwerveModule frontRightModule,
            SwerveModule backLeftModule,
            SwerveModule backRighModule, Pigeon2 pigeon, SwerveParams swerveParams, SwerveSubsystem swerve) {
        modules = (ArrayList<SwerveModule>) List.of(frontLeftModule, frontRightModule, backLeftModule, backRighModule);
        this.pigeon = pigeon;
        this.swerveParams = swerveParams;

        modules.forEach(module -> module.configController(swerveParams));

        pathPlanner = new IPathPlanner() {
        };
        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry
         * readings. The numbers used below are robot specific, and should be
         * tuned.
         */
        poseEstimator = new SwerveDrivePoseEstimator(swerveParams.getKinematics(),
                getRawGyroHeading(),
                getModulePositions(),
                new Pose2d(),
                swerveParams.getModulePoseEstStdDevs().toVector(),
                swerveParams.getVisionPoseEstStdDevs().toVector());
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
        var props = swerveParams.getSpeedLimiterProps();
        var driveProp = creep ? props.slowDriveProp : props.fastDriveProp;
        var steerProp = creep ? props.slowSteerProp : props.fastSteerProp;
        var maxes = swerveParams.getMaxKinematics();
        var modMaxSpeed = driveProp * maxes.vel;
        var modMaxAngularSpeed = steerProp * maxes.angVel;
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
        SwerveModuleState[] swerveModuleStates = swerveParams.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        if (RobotBase.isSimulation()) {
            var chassisSpeeds = swerveParams.getKinematics().toChassisSpeeds(states);
            simGyroAngleCache = simGyroAngleCache.plus(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.020));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, swerveParams.getMaxKinematics().vel);
        for (int i = 0; i < NUM_MODULES; i++) {
            modules.get(i).setDesiredState(states[i], isOpenLoop);
        }
    }

    public void setVoltageToZero() {
        for (int i = 0; i < NUM_MODULES; i++) {
            modules.get(i).setDriveVoltageForCharacterization(0);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds() {
        return swerveParams.getKinematics().toChassisSpeeds(getSwerveModuleStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            positions[i] = modules.get(i).getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = modules.get(i).getState();
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

    public Command pathfindToPose(Pose2d endPose, PathConstraints pathConstraints, double endVelocity) {
        return pathPlanner.pathfindToPose(endPose, pathConstraints, endVelocity);
    }

    public Command followPath(PathPlannerPath path) {
        return pathPlanner.followPath(path);
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
        return (SwerveModule[]) modules.toArray();
    }
}
