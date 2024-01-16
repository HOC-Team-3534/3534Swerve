package swerve.motors.rev;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.SwerveConstants;
import swerve.motors.IDriveController;

public class NeoDriveController implements IDriveController {
    final CANSparkMax driveMotor;
    final RelativeEncoder driveEncoder;
    final SparkPIDController drivePID;
    final SimpleMotorFeedforward m_driveFeedforward = SwerveConstants
            .SlotConfigs2SimpleMotorFeedForward(SwerveConstants.driveSlotConfigs);

    public NeoDriveController(CANSparkMax driveMotor) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveMotor.getEncoder();
        this.drivePID = driveMotor.getPIDController();
    }

    @Override
    public void config() {
        driveMotor.restoreFactoryDefaults();
        var currentLimits = SwerveConstants.swerveCurrentLimitConfigs.drive;
        driveMotor.setSmartCurrentLimit((int) currentLimits.SupplyCurrentLimit);
        driveMotor.setSecondaryCurrentLimit((int) currentLimits.SupplyCurrentThreshold);
        driveMotor.setInverted(
                SwerveConstants.moduleConfiguration.driveMotorInvert.equals(InvertedValue.CounterClockwise_Positive));
        driveMotor.setIdleMode((SwerveConstants.neutralModes.drive == NeutralModeValue.Brake) ? IdleMode.kBrake
                : IdleMode.kCoast);
        driveMotor.setOpenLoopRampRate(SwerveConstants.driveRampingConfigs.openLoopConfigs.DutyCycleOpenLoopRampPeriod);
        driveMotor.setClosedLoopRampRate(
                SwerveConstants.driveRampingConfigs.closedLoopConfigs.DutyCycleClosedLoopRampPeriod);
        driveEncoder.setPositionConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio
                * SwerveConstants.moduleConfiguration.wheelCircumference);
        driveEncoder.setVelocityConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio
                * SwerveConstants.moduleConfiguration.wheelCircumference / 60.0);
        driveEncoder.setPosition(0);
        drivePID.setP(SwerveConstants.driveSlotConfigs.kP);
    }

    @Override
    public double getVoltage() {
        return driveMotor.getAppliedOutput();
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxKinematics.vel;
            driveMotor.set(percentOutput);
        } else {
            drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0,
                    m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }
}
