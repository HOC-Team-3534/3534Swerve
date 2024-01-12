package swerve;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Custom PathPlanner version of SwerveControllerCommand */
public class MyPPSwerveControllerCommand extends Command {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final Runnable resetController;
  private final Consumer<SwerveModuleState[]> outputModuleStates;

  private com.pathplanner.lib.path.PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError = MyPPSwerveControllerCommand::defaultLogError;

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the
   * provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the output to zero upon completion of
   * the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param poseSupplier       A function that supplies the robot pose - use one
   *                           of the odometry classes
   *                           to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param controller         The holonomic driver controller
   * @param resetController    Method to reset holonomic drive controller
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param useAllianceColor   Should the path states be automatically transformed
   *                           based on alliance
   *                           color? In order for this to work properly, you MUST
   *                           create your path on the blue side of
   *                           the field.
   * @param requirements       The subsystems to require.
   */
  public MyPPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Runnable resetController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = controller;
    this.resetController = resetController;
    this.outputModuleStates = outputModuleStates;

    addRequirements(requirements);
  }

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the
   * provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the output to zero upon completion of
   * the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param poseSupplier       A function that supplies the robot pose - use one
   *                           of the odometry classes
   *                           to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param controller         The holonomic driver controller
   * @param resetController    Method to reset holonomic drive controller
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  public MyPPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Runnable resetController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        kinematics,
        controller,
        resetController,
        outputModuleStates,
        false,
        requirements);
  }

  @Override
  public void initialize() {
    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(trajectory);
    }

    timer.reset();
    timer.start();

    PathPlannerServer.sendActivePath(trajectory.getStates());

    resetController.run();
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    State desiredState = (State) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = this.poseSupplier.get();

    PathPlannerServer.sendPathFollowingData(
        desiredState.getTargetHolonomicPose(),
        currentPose);

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState.getDifferentialPose(),desiredState.velocityMps,
        desiredState.targetHolonomicRotation);

    SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);
    this.outputModuleStates.accept(targetModuleStates);

    if (logTargetPose != null) {
      logTargetPose.accept(
          new Pose2d(desiredState.positionMeters, desiredState.targetHolonomicRotation));
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.positionMeters),
          currentPose.getRotation().minus(desiredState.targetHolonomicRotation));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMps) < 0.1) {

      this.outputModuleStates.accept(
          this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));

    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default
   * configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory
   *                            representing the
   *                            active path. This will be called whenever a
   *                            PPSwerveControllerCommand starts
   * @param logTargetPose       Consumer that accepts a Pose2d representing the
   *                            target pose while path
   *                            following
   * @param logSetpoint         Consumer that accepts a ChassisSpeeds object
   *                            representing the setpoint
   *                            speeds
   * @param logError            BiConsumer that accepts a Translation2d and
   *                            Rotation2d representing the error
   *                            while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    MyPPSwerveControllerCommand.logActiveTrajectory = logActiveTrajectory;
    MyPPSwerveControllerCommand.logTargetPose = logTargetPose;
    MyPPSwerveControllerCommand.logSetpoint = logSetpoint;
    MyPPSwerveControllerCommand.logError = logError;
  }
}
