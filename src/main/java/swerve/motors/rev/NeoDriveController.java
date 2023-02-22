package swerve.motors.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.SwerveConstants;
import swerve.motors.IDriveController;

public class NeoDriveController implements IDriveController {
    final CANSparkMax driveMotor;
    final RelativeEncoder driveEncoder;
    final SparkMaxPIDController drivePID;
    final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
            SwerveConstants.driveKV,
            SwerveConstants.driveKA);

    public NeoDriveController(CANSparkMax driveMotor) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveMotor.getEncoder();
        this.drivePID = driveMotor.getPIDController();
    }

    @Override
    public void config() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        driveMotor.setSecondaryCurrentLimit(SwerveConstants.drivePeakCurrentLimit);
        driveMotor.setInverted(SwerveConstants.moduleConfiguration.driveMotorInvert);
        driveMotor.setIdleMode(SwerveConstants.driveIdleMode);
        driveMotor.setOpenLoopRampRate(SwerveConstants.openLoopRamp);
        driveMotor.setClosedLoopRampRate(SwerveConstants.closedLoopRamp);
        driveEncoder.setPositionConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio
                * SwerveConstants.moduleConfiguration.wheelCircumference);
        driveEncoder.setVelocityConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio
                * SwerveConstants.moduleConfiguration.wheelCircumference / 60.0);
        driveEncoder.setPosition(0);
        drivePID.setP(SwerveConstants.driveKP);
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
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0,
                    m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                : 1));
    }
}
