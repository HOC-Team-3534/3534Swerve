package swerve.params;

public class SpeedLimiterProportions {
    public final double fastDriveProp, fastSteerProp, slowDriveProp, slowSteerProp;

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
