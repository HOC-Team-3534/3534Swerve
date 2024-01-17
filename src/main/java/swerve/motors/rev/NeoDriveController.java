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
import swerve.motors.IDriveController;
import swerve.params.SwerveParams;

public class NeoDriveController implements IDriveController {
    final CANSparkMax driveMotor;
    final RelativeEncoder driveEncoder;
    final SparkPIDController drivePID;
    SimpleMotorFeedforward m_driveFeedforward;
    SwerveParams swerveParams;

    public NeoDriveController(CANSparkMax driveMotor) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveMotor.getEncoder();
        this.drivePID = driveMotor.getPIDController();
    }

    @Override
    public void config(SwerveParams swerveParams) {
        this.swerveParams = swerveParams;
        var currentLimits = swerveParams.getSwerveCurrentLimitConfigs().drive;
        var driveRamps = swerveParams.getDriveRampingConfigs();
        var modConfig = swerveParams.getModuleConfiguration();
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit((int) currentLimits.SupplyCurrentLimit);
        driveMotor.setSecondaryCurrentLimit((int) currentLimits.SupplyCurrentThreshold);
        driveMotor.setInverted(modConfig.driveMotorInvert.equals(InvertedValue.CounterClockwise_Positive));
        driveMotor.setIdleMode((swerveParams.getNeutralModes().drive == NeutralModeValue.Brake) ? IdleMode.kBrake
                : IdleMode.kCoast);
        driveMotor.setOpenLoopRampRate(driveRamps.openLoopConfigs.DutyCycleOpenLoopRampPeriod);
        driveMotor.setClosedLoopRampRate(driveRamps.closedLoopConfigs.DutyCycleClosedLoopRampPeriod);
        driveEncoder.setPositionConversionFactor(1 / modConfig.driveGearRatio * modConfig.wheelCircumference);
        driveEncoder.setVelocityConversionFactor(1 / modConfig.driveGearRatio * modConfig.wheelCircumference / 60.0);
        driveEncoder.setPosition(0);
        drivePID.setP(swerveParams.getDriveSlotConfigs().kP);

        m_driveFeedforward = SwerveParams.SlotConfigs2SimpleMotorFeedForward(swerveParams.getDriveSlotConfigs());
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
            double percentOutput = desiredState.speedMetersPerSecond / swerveParams.getMaxKinematics().vel;
            driveMotor.set(percentOutput);
        } else {
            drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0,
                    m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((swerveParams.getModuleConfiguration().driveMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }
}
