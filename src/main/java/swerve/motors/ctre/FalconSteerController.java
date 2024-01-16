package swerve.motors.ctre;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.Conversions;
import swerve.SwerveConstants;
import swerve.motors.ISteerController;

public class FalconSteerController implements ISteerController {
    final TalonFX steerMotor;
    final CANcoder absoluteEncoder;
    final Rotation2d angleOffset;
    double resetIteration;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    Rotation2d lastAngle;
    private static final double SpeedNotChangeAngle = 0.05;

    public FalconSteerController(TalonFX steerMotor, CANcoder absoluteEncoder,
            Rotation2d angleOffset) {
        this.steerMotor = steerMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.angleOffset = angleOffset;
    }

    @Override
    public void config() {
        absoluteEncoder.getConfigurator().apply(SwerveConstants.swerveCanCoderConfig);

        steerMotor.getConfigurator().apply(SwerveConstants.swerveAngleFXconfig);

        resetToAbsolute();
        lastAngle = getAngle();
    }

    Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d
                .fromRotations(Conversions.falconRotationsToWheelRotations(steerMotor.getPosition().getValueAsDouble(),
                        SwerveConstants.moduleConfiguration.angleGearRatio));
    }

    @Override
    public void setVoltage(double voltage) {
        if (Math.abs(getRate().getRadians()) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                resetToAbsolute();
            }
        } else {
            resetIteration = 0;
        }
        steerMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.angleMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.steerDegreesToFalconRotations(
                getCanCoder().getDegrees() - angleOffset.getDegrees(),
                SwerveConstants.moduleConfiguration.angleGearRatio);
        steerMotor.setPosition(absolutePosition);
    }

    @Override
    public Rotation2d getRate() {
        return Rotation2d
                .fromRotations(Conversions.falconRotationsToWheelRotations(steerMotor.getVelocity().getValueAsDouble(),
                        SwerveConstants.moduleConfiguration.angleGearRatio));
    }

    @Override
    public double getVoltage() {
        return steerMotor.getMotorVoltage().getValueAsDouble() * ((SwerveConstants.moduleConfiguration.angleMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= SpeedNotChangeAngle) ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steerMotor.setControl(new PositionDutyCycle(Conversions.steerDegreesToFalconRotations(angle.getDegrees(),
                SwerveConstants.moduleConfiguration.angleGearRatio)));
        lastAngle = angle;
    }
}
