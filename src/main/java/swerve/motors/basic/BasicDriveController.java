package swerve.motors.basic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import swerve.motors.IDriveController;
import swerve.params.SwerveParams;

public class BasicDriveController implements IDriveController {
    final MotorController driveMotor;
    final Encoder driveEncoder;
    final int kEncoderResolution = 4096;
    double lastVoltage;
    PIDController m_drivePIDController;
    SimpleMotorFeedforward m_driveFeedforward;
    SwerveParams swerveParams;

    public BasicDriveController(MotorController driveMotor, Encoder driveEncoder) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
    }

    @Override
    public void config(SwerveParams swerveParams) {
        this.swerveParams = swerveParams;

        var modConfig = swerveParams.getModuleConfiguration();
        var driveSlot = swerveParams.getDriveSlotConfigs();

        driveEncoder
                .setDistancePerPulse(Math.PI * modConfig.wheelDiameter / modConfig.driveGearRatio / kEncoderResolution);

        m_drivePIDController = new PIDController(driveSlot.kP, 0, 0);
        m_driveFeedforward = SwerveParams.SlotConfigs2SimpleMotorFeedForward(driveSlot);
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
            double percentOutput = desiredState.speedMetersPerSecond / swerveParams.getMaxKinematics().vel;
            driveMotor.setVoltage(percentOutput);
        } else {
            // Calculate the drive output from the drive PID controller.
            double driveOutput = m_drivePIDController.calculate(getVelocity(), desiredState.speedMetersPerSecond);
            double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            setVoltage(driveOutput + driveFeedforward);
        }
    }
}