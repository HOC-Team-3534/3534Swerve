package swerve.motors.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.Conversions;
import swerve.SwerveConstants;
import swerve.motors.ISteerController;

public class FalconSteerController implements ISteerController {
    final WPI_TalonFX steerMotor;
    final CANCoder absoluteEncoder;
    final Rotation2d angleOffset;
    double resetIteration;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    Rotation2d lastAngle;
    private static final double SpeedNotChangeAngle = 0.05;

    public FalconSteerController(WPI_TalonFX steerMotor, CANCoder absoluteEncoder,
            Rotation2d angleOffset) {
        this.steerMotor = steerMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.angleOffset = angleOffset;
    }

    @Override
    public void config() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(SwerveConstants.swerveCanCoderConfig);
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(SwerveConstants.swerveAngleFXconfig);
        steerMotor.setInverted(SwerveConstants.moduleConfiguration.angleMotorInvert);
        steerMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
        resetToAbsolute();
        lastAngle = getAngle();
    }

    Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
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
        steerMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                : 1));
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
                SwerveConstants.moduleConfiguration.angleGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public Rotation2d getRate() {
        return Rotation2d.fromDegrees(Conversions.falconToDegreesPerSecond(steerMotor.getSelectedSensorVelocity(),
                SwerveConstants.moduleConfiguration.angleGearRatio));
    }

    @Override
    public double getVoltage() {
        return steerMotor.getMotorOutputVoltage() * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                : 1);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= SpeedNotChangeAngle) ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steerMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.moduleConfiguration.angleGearRatio));
        lastAngle = angle;
    }
}
