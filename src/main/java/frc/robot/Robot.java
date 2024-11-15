// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Arm.*;

public class Robot extends TimedRobot {
  private final Joystick m_driveController = new Joystick(0);
  // now in arm also
  private final Joystick m_auxController = new Joystick(1);

  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm(); 
  private Auto m_auto = new Auto(m_swerve, m_arm);

  // Slew rate limiters to make joystick inputs more gentle; 1/5 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  //private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);


  @Override
  public void robotInit(){
    m_swerve.resetGyroYaw();
    m_swerve.resetPosition();

    //Happy with PID values for now
    //Constants.addPIFtoSmartDashboard();
    
  }

  @Override
  public void robotPeriodic() {
    m_swerve.updateOdometry();
    m_auto.choose(); 
    SmartDashboard.putNumber("Gyro: ", m_swerve.getHeading());
    SmartDashboard.putNumber("Crank Position: ", m_arm.getCrankPosition());
    SmartDashboard.putNumber("Desired Crank Position", m_arm.getDesiredCrankPosition());
    SmartDashboard.putBoolean("Have Note? ", m_arm.hasNote());
    SmartDashboard.putString("State: ", m_arm.getState());

    //Happy with PID values for now
    //Constants.updatePID();
    //m_swerve.updatePIDValues();
    //m_arm.updatePIDValues();

  }

  @Override
  public void autonomousInit() {
    m_swerve.resetGyroYaw();
    m_swerve.resetPosition();
    m_auto.selectAuto();
    m_auto.resetTimer();
  }

  @Override
  public void autonomousPeriodic() {
    m_auto.setPeriod(getPeriod());
    m_auto.runSelectedAuto();
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
    m_arm.setState(ArmState.DEFAULT);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    m_arm.moveCrank();

    if(m_driveController.getRawButton(7))
    {
      m_arm.setClimbLock(0);
    }
    else
    {
      m_arm.setClimbLock(1);
    }

    if(m_driveController.getRawButton(1))
    {
      m_swerve.resetGyroYaw();
    }

    if(m_auxController.getRawButton(1))
      m_arm.setState(ArmState.SCORE_AMP);

    else if(m_auxController.getRawButton(2))
      m_arm.setState(ArmState.COLLECT_FROM_GROUND);

    else if(m_auxController.getRawButton(3))  
      m_arm.setState(ArmState.COLLECT_FROM_SOURCE);

    else if(m_auxController.getRawButton(4))
     m_arm.setState(ArmState.SCORE_SPEAKER);

    else if(m_auxController.getRawButton(8))
      m_arm.setState(ArmState.DEFAULT);

    if(m_auxController.getRawAxis(2)>.1)
      m_arm.runShooter(ArmConstants.FORWARD);
    else if(m_auxController.getRawButton(5))
      m_arm.runShooter(ArmConstants.REVERSE);
    else if(m_auxController.getRawAxis(3) > .1)
      m_arm.runIntake(ArmConstants.FORWARD);
    else if(m_auxController.getRawButton(6))
      m_arm.runIntake((ArmConstants.REVERSE));
    else if(m_auxController.getPOV() != -1    && 
            (m_auxController.getPOV() >= 315  || 
            m_auxController.getPOV() <= 45))
      m_arm.Purge(ArmConstants.FORWARD);
    else if(m_auxController.getPOV() != -1  && 
            m_auxController.getPOV() >= 135 && 
            m_auxController.getPOV() <= 225)
      m_arm.Purge(ArmConstants.REVERSE);
  
    else
      m_arm.stopAll();

      if(m_arm.shouldRumble())
      {
        m_driveController.setRumble(RumbleType.kBothRumble, 0.5);
        m_auxController.setRumble(RumbleType.kBothRumble, 0.5);
      }
      else
      {
        m_driveController.setRumble(RumbleType.kBothRumble, 0);
        m_auxController.setRumble(RumbleType.kBothRumble, 0);
      }
  
  }

  private void driveWithJoystick(boolean fieldRelative) {

    final double robotSpeed;

    if(m_driveController.getRawButton(6))
      robotSpeed = DrivetrainConstants.SLOW_DRVE_SPEED;
    else
      robotSpeed = DrivetrainConstants.MAX_DRIVE_SPEED;
      

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.1))
            * robotSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.1))
            * robotSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.1)
            * DrivetrainConstants.MAX_ANGLE_SPEED;

    m_swerve.drive(xSpeed, ySpeed, robotSpeed, rot, fieldRelative, getPeriod());
  }
}
