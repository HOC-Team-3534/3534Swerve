package swerve.motors.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import swerve.SwerveConstants;
import swerve.motors.IDriveController;

public class BasicDriveController implements IDriveController {
    final MotorController driveMotor;
    final Encoder driveEncoder;
    final int kEncoderResolution = 4096;
    double lastVoltage;
    final PIDController m_drivePIDController = new PIDController(SwerveConstants.driveKP, 0, 0);
    final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
            SwerveConstants.driveKV,
            SwerveConstants.driveKA);

    public BasicDriveController(MotorController driveMotor, Encoder driveEncoder) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
    }

    @Override
    public void config() {
        var config = SwerveConstants.moduleConfiguration;
        driveEncoder.setDistancePerPulse(Math.PI * config.wheelDiameter / config.driveGearRatio / kEncoderResolution);
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getRate();
    }

    @Override
    public double getDistance() {
        return driveEncoder.getDistance();
    }

    @Override
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
        lastVoltage = voltage;
    }

    @Override
    public double getVoltage() {
        return lastVoltage;
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            driveMotor.setVoltage(percentOutput);
        } else {
            // Calculate the drive output from the drive PID controller.
            double driveOutput = m_drivePIDController.calculate(getVelocity(), desiredState.speedMetersPerSecond);
            double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            setVoltage(driveOutput + driveFeedforward);
        }
    }
}