// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PID;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = 2*Constants.DrivetrainConstants.MAX_ANGLE_SPEED;
  private static final double kModuleMaxAngularAcceleration =
      8 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(
          PID.kP_drive, 
          PID.kI_drive, 
          PID.kD_drive);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          PID.kP_turn,
          PID.kI_turn,
          PID.kD_turn,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.25, 0.25);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.25, 0.25);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANcoderChannel) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveMotor.setInverted(true);
    m_turningMotor.setInverted(true);
    m_turningEncoder = new CANcoder(CANcoderChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_driveEncoder.setPositionConversionFactor(2*Math.PI * ModuleConstants.kWheelRadius / ModuleConstants.kDriveMotorGearRatio);
    m_driveEncoder.setVelocityConversionFactor(2*Math.PI * ModuleConstants.kWheelRadius/ ModuleConstants.kDriveMotorGearRatio / 60.0);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.CA
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    //CANcoderConfigurator configurator = m_turningEncoder.getConfigurator();
    //MagnetSensorConfigs configs = new MagnetSensorConfigs();
    //configurator.apply(configs);


    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(directionInRads()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(directionInRads()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(directionInRads());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(directionInRads(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput /*+ driveFeedforward*/);
    m_turningMotor.setVoltage(turnOutput /*+ turnFeedforward*/);
  }

  public double directionInRads()
  {
m_turningEncoder.getPosition();

    return m_turningEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
  
  }

  public void updateModulePIDValues()
  {
    m_drivePIDController.setP(Constants.PID.kP_drive);
    m_drivePIDController.setI(Constants.PID.kI_drive);
    m_drivePIDController.setD(Constants.PID.kD_drive);

    m_turningPIDController.setP(Constants.PID.kP_turn);
    m_turningPIDController.setI(Constants.PID.kI_turn);
    m_turningPIDController.setD(Constants.PID.kD_turn);

  }

}
