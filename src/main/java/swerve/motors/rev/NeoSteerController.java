package swerve.motors.rev;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.SwerveConstants;
import swerve.motors.ISteerController;

public class NeoSteerController implements ISteerController {
    final CANSparkMax steerMotor;
    final RelativeEncoder steerEncoder;
    final SparkMaxPIDController steerPID;
    final CANCoder absoluteEncoder;
    final Rotation2d angleOffset;
    Rotation2d lastAngle;
    private static final double SpeedNotChangeAngle = 0.05;

    public NeoSteerController(CANSparkMax steerMotor, CANCoder absoluteEncoder,
            Rotation2d angleOffset) {
        this.steerMotor = steerMotor;
        this.steerEncoder = steerMotor.getEncoder();
        this.steerPID = steerMotor.getPIDController();
        this.absoluteEncoder = absoluteEncoder;
        this.angleOffset = angleOffset;
    }

    @Override
    public void config() {
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(SwerveConstants.swerveCanCoderConfig);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        steerMotor.setSecondaryCurrentLimit(SwerveConstants.anglePeakCurrentLimit);
        steerMotor.setInverted(SwerveConstants.moduleConfiguration.angleMotorInvert);
        steerMotor.setIdleMode(SwerveConstants.angleIdleMode);
        steerEncoder.setPositionConversionFactor(1 / SwerveConstants.moduleConfiguration.angleGearRatio * 360.0);
        steerEncoder.setVelocityConversionFactor(1 / SwerveConstants.moduleConfiguration.angleGearRatio * 360.0 / 60.0);
        resetToAbsolute();
        steerPID.setP(SwerveConstants.steerKP);
        steerPID.setI(SwerveConstants.steerKI);
        steerPID.setD(SwerveConstants.steerKD);
        steerPID.setFF(SwerveConstants.steerKF);
        lastAngle = getAngle();
    }

    public void resetToAbsolute() {
        steerEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= SpeedNotChangeAngle) ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less than 1%. Prevents Jittering.
        steerPID.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    @Override
    public double getVoltage() {
        return steerMotor.getAppliedOutput();
    }

    @Override
    public Rotation2d getRate() {
        return Rotation2d.fromDegrees(steerEncoder.getVelocity());
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }

    @Override
    public void setVoltage(double voltage) {
        steerMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                : 1));
    }
}
