// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import swerve.motors.IDriveController;
import swerve.motors.ISteerController;
import swerve.motors.basic.BasicDriveController;
import swerve.motors.basic.BasicSteerController;
import swerve.motors.ctre.FalconDriveController;
import swerve.motors.ctre.FalconSteerController;
import swerve.motors.rev.NeoDriveController;
import swerve.motors.rev.NeoSteerController;

public class SwerveModule {
  final IDriveController m_driveController;
  final ISteerController m_steerController;
  final ModuleType moduleType;
  /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
  private double simSpeedCache, simPositionCache;
  private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

  enum ModuleType {
    FalconFalconCANcoder, FalconNEOCANcoder, NEOFalconCANcoder, NEONEOCANcoder, Basic
  }

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotor
   *          PWM drive motor
   * @param turningMotor
   *          PWM turning motor
   * @param driveEncoder
   *          DIO drive encoder
   * @param turningEncoder
   *          DIO turning encoder
   */
  public SwerveModule(MotorController driveMotor, MotorController turningMotor,
      Encoder driveEncoder, Encoder turningEncoder) {
    m_driveController = new BasicDriveController(driveMotor, driveEncoder);
    m_steerController = new BasicSteerController(turningMotor, turningEncoder);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.Basic;
  }

  public SwerveModule(TalonFX driveMotor, TalonFX steerMotor,
      CANcoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new FalconDriveController(driveMotor);
    m_steerController = new FalconSteerController(steerMotor, absoluteEncoder,
        angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconFalconCANcoder;
  }

  public SwerveModule(TalonFX driveMotor, CANSparkMax steerMotor,
      CANcoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new FalconDriveController(driveMotor);
    m_steerController = new NeoSteerController(steerMotor, absoluteEncoder, angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconNEOCANcoder;
  }

  public SwerveModule(CANSparkMax driveMotor, TalonFX steerMotor,
      CANcoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new NeoDriveController(driveMotor);
    m_steerController = new FalconSteerController(steerMotor, absoluteEncoder,
        angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.NEOFalconCANcoder;
  }

  public SwerveModule(CANSparkMax driveMotor, CANSparkMax steerMotor,
      CANcoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new NeoDriveController(driveMotor);
    m_steerController = new NeoSteerController(steerMotor, absoluteEncoder, angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconNEOCANcoder;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState((RobotBase.isReal()) ? m_driveController.getVelocity()
        : simSpeedCache,
        (RobotBase.isReal()) ? m_steerController.getAngle()
            : simAngleCache);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    simPositionCache += simSpeedCache * 0.020;
    return new SwerveModulePosition((RobotBase.isReal()) ? m_driveController.getDistance()
        : simPositionCache,
        (RobotBase.isReal()) ? m_steerController.getAngle()
            : simAngleCache);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState
   *          Desired state with speed and angle.
   * @param isOpenLoop
   *          use percent output instead of velocity control.
   */
  public void setDesiredState(SwerveModuleState desiredState,
      boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    switch (moduleType) {
      case Basic:
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        break;

      default:
        // should work for REV too
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        break;
    }
    m_driveController.setSpeed(desiredState, isOpenLoop);
    m_steerController.setAngle(desiredState);
    simSpeedCache = desiredState.speedMetersPerSecond;
    simAngleCache = desiredState.angle;
  }

  /**
   * Use for characterizing the drive only. Steer set to 0
   * 
   * @param voltage
   *          the voltage to set the drive motor to
   */
  public void setDriveVoltageForCharacterization(double voltage) {
    m_driveController.setVoltage(voltage);
    switch (moduleType) {
      case FalconFalconCANcoder:
        // need to send more than 1% speed otherwise it wont set the angle b/c of the
        // nice
        // filter to reduce jittering, but it doesnt actually set the speed for drive
        // to 1 meter per second
        m_steerController.setAngle(new SwerveModuleState(1, new Rotation2d()));
        break;

      default:
        m_steerController.setVoltage(0);
        break;
    }
  }

  /**
   * Use for characterizing a steering motor only. Drive set to 0
   * 
   * @param voltage
   *          the voltage to set the steer motor to
   */
  public void setSteerVoltageForCharacterization(double voltage) {
    m_steerController.setVoltage(voltage);
    m_driveController.setVoltage(0);
  }

  /**
   * @return the drive motor voltage and velocity in meters per second of the
   *         wheel
   */
  public Pair<Double, Double> getDriveVoltageAndRate() {
    return new Pair<>(m_driveController.getVoltage(),
        m_driveController.getVelocity());
  }

  public void resetToAbsolute() {
    m_steerController.resetToAbsolute();
  }

  /**
   * @return the steer motor voltage and angular velocity in radians per second
   *         of the wheel vertical axel
   */
  public Pair<Double, Double> getSteerVoltageAndRate() {
    return new Pair<>(m_steerController.getVoltage(),
        m_steerController.getRate().getRadians());
  }
}
