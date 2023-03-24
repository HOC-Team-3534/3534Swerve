package swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StraightToPoseCommand extends CommandBase {
    final Pose2d endPose;
    final ProfiledPIDController xController, yController;
    final HolonomicDriveController holo;
    final Translation2d translationTolerance;
    final Rotation2d rotationTolerance;
    final Supplier<Pose2d> poseSupplier;
    final SwerveDriveKinematics kinematics;
    final Runnable resetController;
    final Consumer<SwerveModuleState[]> outputModuleStates;

    public StraightToPoseCommand(Pose2d endPose, TrapezoidProfile.Constraints drivConstraints,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics, HolonomicDriveController holo, Runnable resetController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Translation2d translationTolerance, Rotation2d rotationalTolerance, Subsystem... requirements) {
        this.endPose = endPose;
        this.xController = new ProfiledPIDController(0, 0, 0, drivConstraints);
        this.yController = new ProfiledPIDController(0, 0, 0, drivConstraints);
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.holo = holo;
        this.resetController = resetController;
        this.outputModuleStates = outputModuleStates;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationalTolerance;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        resetController.run();

        var currentPose = poseSupplier.get();

        xController.reset(currentPose.getX());
        xController.setGoal(endPose.getX());
        xController.setTolerance(translationTolerance.getX());
        yController.reset(currentPose.getY());
        yController.setGoal(endPose.getY());
        yController.setTolerance(translationTolerance.getY());

        holo.getThetaController().setTolerance(rotationTolerance.getRadians());
    }

    @Override
    public void execute() {
        var currentPose = poseSupplier.get();

        xController.calculate(currentPose.getX());
        yController.calculate(currentPose.getY());

        var xSetPoint = xController.getSetpoint();
        var ySetPoint = yController.getSetpoint();

        /**
         * Unfortunately have to do a little work to use the available holonomic
         * controller... ugh, but itll be okay
         */

        var velocities = new Translation2d(xSetPoint.velocity, ySetPoint.velocity);

        var desiredPose = new Pose2d(xSetPoint.position, ySetPoint.position,
                velocities.getAngle());

        var targetChassisSpeeds = this.holo.calculate(currentPose, desiredPose, velocities.getNorm(),
                endPose.getRotation());

        var targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);
        this.outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        this.outputModuleStates.accept(
                this.kinematics.toSwerveModuleStates(new ChassisSpeeds()));

    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && holo.getThetaController().atGoal();
    }

}
