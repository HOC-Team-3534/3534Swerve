package swerve.motors.basic;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import swerve.SwerveConstants;
import swerve.motors.ISteerController;

public class BasicSteerController implements ISteerController {
    final MotorController steerMotor;
    final Encoder steerEncoder;
    final int kEncoderResolution = 4096;
    double lastVoltage;
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.steerKS,
            SwerveConstants.steerKV);
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(SwerveConstants.steerKP, 0,
            0,
            new TrapezoidProfile.Constraints(SwerveConstants.moduleMaxAngularVel,
                    SwerveConstants.moduleMaxAngularAccel));

    public BasicSteerController(MotorController steerMotor, Encoder steerEncoder) {
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
    }

    @Override
    public void config() {
        var config = SwerveConstants.moduleConfiguration;
        steerEncoder.setDistancePerPulse(2 * Math.PI / config.angleGearRatio / kEncoderResolution);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getDistance());
    }

    @Override
    public void setVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
        lastVoltage = voltage;
    }

    @Override
    public Rotation2d getRate() {
        return new Rotation2d(steerEncoder.getRate());
    }

    @Override
    public double getVoltage() {
        return lastVoltage;
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getAngle().getRadians(),
                desiredState.angle.getRadians());
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
        setVoltage(turnOutput + turnFeedforward);
    }

    @Override
    public void resetToAbsolute() {
        // No Absolute Encoder
    }
}
