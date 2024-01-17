package swerve.path;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IPathPlanner {
    default void configureHolonomic(Supplier<Pose2d> poseSupplier, Consumer<Pose2d> resetPose,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier, Consumer<ChassisSpeeds> robotRelativeOutput,
            HolonomicPathFollowerConfig config, BooleanSupplier shouldFlipPath, Subsystem driveSubsystem) {
        AutoBuilder.configureHolonomic(poseSupplier, resetPose, robotRelativeSpeedsSupplier,
                robotRelativeOutput, config, shouldFlipPath, driveSubsystem);
    }

    default Command pathfindToPose(Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity);
    }

    default Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }
}
