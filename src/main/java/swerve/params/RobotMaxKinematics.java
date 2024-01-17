package swerve.params;

public class RobotMaxKinematics {
    public final double vel, angVel, angAccel;

    public RobotMaxKinematics(double vel, double angVel, double angAccel) {
        this.vel = vel;
        this.angVel = angVel;
        this.angAccel = angAccel;
    }
}
