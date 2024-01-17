package swerve.motors.ctre;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerve.Conversions;
import swerve.motors.IDriveController;
import swerve.params.SwerveParams;

public class FalconDriveController implements IDriveController {
    final TalonFX driveMotor;
    SwerveParams swerveParams;

    public FalconDriveController(TalonFX driveMotor) {
        this.driveMotor = driveMotor;
    }

    @Override
    public void config(SwerveParams swerveParams) {
        this.swerveParams = swerveParams;

        driveMotor.getConfigurator().apply(swerveParams.getDriveFxConfiguration());
        driveMotor.setPosition(0);
    }

    @Override
    public double getVelocity() {
        var modConfig = swerveParams.getModuleConfiguration();
        return Conversions.falconRotationsPerSecondToWheelMetersPerSecond(driveMotor.getVelocity().getValueAsDouble(),
                modConfig.wheelCircumference,
                modConfig.driveGearRatio);
    }

    @Override
    public double getDistance() {
        var modConfig = swerveParams.getModuleConfiguration();
        return Conversions.falconRotationsToWheelMeters(driveMotor.getPosition().getValueAsDouble(),
                modConfig.wheelCircumference,
                modConfig.driveGearRatio);
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage * ((swerveParams.getModuleConfiguration().driveMotorInvert
                .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                        : 1));
    }

    @Override
    public double getVoltage() {
        return driveMotor.getMotorVoltage().getValueAsDouble()
                * ((swerveParams.getModuleConfiguration().driveMotorInvert
                        .equals(InvertedValue.CounterClockwise_Positive)) ? -1
                                : 1);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / swerveParams.getMaxKinematics().vel;
            driveMotor.set(percentOutput);
        } else {
            var modConfig = swerveParams.getModuleConfiguration();
            double velocity = Conversions.wheelMetersPerSecondToFalconRotationsPerSecond(
                    desiredState.speedMetersPerSecond, modConfig.wheelCircumference,
                    modConfig.driveGearRatio);
            driveMotor.setControl(new VelocityDutyCycle(velocity));
        }
    }
}