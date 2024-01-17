package swerve.motors.rev;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.motors.ISteerController;
import swerve.params.SwerveParams;

public class NeoSteerController implements ISteerController {
    final CANSparkMax steerMotor;
    final RelativeEncoder steerEncoder;
    final SparkPIDController steerPID;
    final CANcoder absoluteEncoder;
    final Rotation2d angleOffset;
    Rotation2d lastAngle;
    private static final double SpeedNotChangeAngle = 0.05;
    SwerveParams swerveParams;

    public NeoSteerController(CANSparkMax steerMotor, CANcoder absoluteEncoder,
            Rotation2d angleOffset) {
        this.steerMotor = steerMotor;
        this.steerEncoder = steerMotor.getEncoder();
        this.steerPID = steerMotor.getPIDController();
        this.absoluteEncoder = absoluteEncoder;
        this.angleOffset = angleOffset;
    }

    @Override
    public void config(SwerveParams swerveParams) {
        this.swerveParams = swerveParams;

        absoluteEncoder.getConfigurator().apply(swerveParams.getSwerveCanCoderConfig());

        steerMotor.restoreFactoryDefaults();

        var currentLimits = swerveParams.getSwerveCurrentLimitConfigs().steer;
        var modConfig = swerveParams.getModuleConfiguration();

        steerMotor.setSmartCurrentLimit((int) currentLimits.SupplyCurrentLimit);
        steerMotor.setSecondaryCurrentLimit((int) currentLimits.SupplyCurrentThreshold);
        steerMotor.setInverted(modConfig.angleMotorInvert.equals(InvertedValue.CounterClockwise_Positive));
        steerMotor.setIdleMode((swerveParams.getNeutralModes().steer == NeutralModeValue.Brake) ? IdleMode.kBrake
                : IdleMode.kCoast);
        steerEncoder.setPositionConversionFactor(1 / modConfig.angleGearRatio * 360.0);
        steerEncoder.setVelocityConversionFactor(1 / modConfig.angleGearRatio * 360.0 / 60.0);
        resetToAbsolute();
        var anglePID = modConfig.angleSlotConfigs;
        steerPID.setP(anglePID.kP);
        steerPID.setI(anglePID.kI);
        steerPID.setD(anglePID.kD);
        steerPID.setFF(anglePID.kS);
        lastAngle = getAngle();
    }

    public void resetToAbsolute() {
        steerEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
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
        steerMotor.setVoltage(voltage * ((swerveParams.getModuleConfiguration().angleMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }
}
