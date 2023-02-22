package swerve.motors.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.Conversions;
import swerve.SwerveConstants;
import swerve.motors.IDriveController;

public class FalconDriveController implements IDriveController {
    final WPI_TalonFX driveMotor;
    final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
            SwerveConstants.driveKV,
            SwerveConstants.driveKA);

    public FalconDriveController(WPI_TalonFX driveMotor) {
        this.driveMotor = driveMotor;
    }

    @Override
    public void config() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(SwerveConstants.swerveDriveFXConfig);
        driveMotor.setInverted(SwerveConstants.moduleConfiguration.driveMotorInvert);
        driveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }

    @Override
    public double getVelocity() {
        var config = SwerveConstants.moduleConfiguration;
        return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), config.wheelCircumference,
                config.driveGearRatio);
    }

    @Override
    public double getDistance() {
        var config = SwerveConstants.moduleConfiguration;
        return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), config.wheelCircumference,
                config.driveGearRatio);
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                : 1));
    }

    @Override
    public double getVoltage() {
        return driveMotor.getMotorOutputVoltage() * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                : 1);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            var config = SwerveConstants.moduleConfiguration;
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, config.wheelCircumference,
                    config.driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
}