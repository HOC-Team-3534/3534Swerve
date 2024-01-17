package swerve.params;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimationStandardDeviations {
    public final double x, y;
    public final Rotation2d angle;

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

    public Matrix<N3, N1> toVector() {
        return VecBuilder.fill(x, y, angle.getRadians());
    }
}