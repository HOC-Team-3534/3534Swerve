package swerve.motors.ctre;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.Conversions;
import swerve.SwerveConstants;
import swerve.motors.IDriveController;

public class FalconDriveController implements IDriveController {
    final TalonFX driveMotor;
    final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
            SwerveConstants.driveKV,
            SwerveConstants.driveKA);

    public FalconDriveController(TalonFX driveMotor) {
        this.driveMotor = driveMotor;
    }

    @Override
    public void config() {
        driveMotor.getConfigurator().apply(SwerveConstants.swerveDriveFXConfig);
        driveMotor.setPosition(0);
    }

    @Override
    public double getVelocity() {
        var config = SwerveConstants.moduleConfiguration;
        return Conversions.falconRotationsPerSecondToWheelMetersPerSecond(driveMotor.getVelocity().getValueAsDouble(),
                config.wheelCircumference,
                config.driveGearRatio);
    }

    @Override
    public double getDistance() {
        var config = SwerveConstants.moduleConfiguration;
        return Conversions.falconRotationsToWheelMeters(driveMotor.getPosition().getValueAsDouble(),
                config.wheelCircumference,
                config.driveGearRatio);
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }

    @Override
    public double getVoltage() {
        return driveMotor.getMotorVoltage().getValueAsDouble() * ((SwerveConstants.moduleConfiguration.driveMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            var config = SwerveConstants.moduleConfiguration;
            double velocity = Conversions.wheelMetersPerSecondToFalconRotationsPerSecond(
                    desiredState.speedMetersPerSecond, config.wheelCircumference,
                    config.driveGearRatio);
            driveMotor.setControl(new VelocityDutyCycle(velocity, 0, false,
                    m_driveFeedforward.calculate(desiredState.speedMetersPerSecond), 0, true, false, false));
        }
    }
}