// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {


  private final Translation2d m_frontLeftLocation = new Translation2d(0.274, 0.274);
  private final Translation2d m_frontRightLocation = new Translation2d(0.274, -0.274);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.274, 0.274);
  private final Translation2d m_backRightLocation = new Translation2d(-0.274, -0.274);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 22);
  private final SwerveModule m_frontRight = new SwerveModule(5, 6, 26);
  private final SwerveModule m_backLeft = new SwerveModule(3, 4, 24);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 28);

 
  private final AHRS m_gyro = new AHRS();

  private final Field2d m_field = new Field2d();

  ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
  
  GenericEntry xPose = fieldTab.add("Pose x:", 0).getEntry();
  GenericEntry yPose = fieldTab.add("Pose y:", 0).getEntry();
  GenericEntry rotPose = fieldTab.add("Pose rot:", 0).getEntry();
  
  //fieldTab.add("Field: ", m_field);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double speed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    Pose2d pose = m_odometry.getPoseMeters();
    
    xPose.setDouble(pose.getX());
    yPose.setDouble(pose.getY());
    rotPose.setDouble(pose.getRotation().getDegrees());
  }

  public double getHeading()
  {
    return m_gyro.getRotation2d().getDegrees();
  }

  public void resetGyroYaw()
  {
    m_gyro.zeroYaw();
  }

  public void resetPosition()
  {
    m_odometry.resetPosition(m_gyro.getRotation2d(), 
            new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()},
             new Pose2d());
  }

  public double getModule1Direction()
  {
    return m_frontLeft.directionInRads();
  }

  public Pose2d getPose(){

    Pose2d pose = m_odometry.getPoseMeters();
    
    xPose.setDouble(pose.getX());
    yPose.setDouble(pose.getY());
    rotPose.setDouble(pose.getRotation().getDegrees());

    return pose;
  }

  public void updatePIDValues()
  {
    m_backLeft.updateModulePIDValues();
    m_backRight.updateModulePIDValues();
    m_frontLeft.updateModulePIDValues();
    m_frontLeft.updateModulePIDValues();
  }

}
