package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.*;
import frc.robot.Constants.*;

public class Auto {

    private static final String kDefaultAuto = "Do nothing";
    private static final String kRight1 = "Right Start, Score 1, Leave";
    private static final String kLeft1 = "Left Start, Score 1, Leave";
    private static final String kCenter1 = "Center Start, Score 1, Leave";
    private static final String kRight2 = "Right Start, Right Return, Score 2";
    private static final String kRight2Center = "Right Start, Center Return, Score 2";
    private static final String kLeft2 = "Left Start, Left Return, Score 2";
    private static final String kLeft2Center = "Left Start, Center Return, Score 2";
    private static final String kCenter2 = "Center Start, Score 2";
    private static final String kCenter3Left = "Center Start, Score 3, Goes LEFT";
    private static final String kCenter3Right = "Center Start, Score 3, Goes RIGHT";
    private static final String kRedCenter4 = "RED Center Start, Score 4, Goes RIGHT first";
    private static final String kBlueCenter4 = "BLUE Center Start, Score 4, Goes LEFT first";


    private static final String kAutoGenerator = "Lofty's Fun Time!";

    private double auto_x = 0;
    private double auto_y = 0;
    private double auto_heading = 0;

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    //private final SlewRateLimiter m_xAutoSlew = new SlewRateLimiter(2);
    //private final SlewRateLimiter m_yAutoSlew = new SlewRateLimiter(2);

    private final Drivetrain m_swerve;
    private final Arm m_arm;

    private double m_period;
    private double m_delay = 0;

    private Timer autoTimer = new Timer();

    private boolean needToResetGyro;

    private PIDController autoXDrivePID = new PIDController(0.4, 0.01, 0);
    private PIDController autoYDrivePID = new PIDController(0.4, 0.01, 0);

    private final SimpleMotorFeedforward x_driveFeedforward = new SimpleMotorFeedforward(0.25, 0.25);
    private final SimpleMotorFeedforward y_driveFeedforward = new SimpleMotorFeedforward(0.25, 0.25);

public Auto(Drivetrain d, Arm a){
    m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
    m_chooser.addOption(kRight1, kRight1);
    m_chooser.addOption(kLeft1, kLeft1);
    m_chooser.addOption(kCenter1, kCenter1);
    m_chooser.addOption(kRight2, kRight2);
    m_chooser.addOption(kRight2Center, kRight2Center);
    m_chooser.addOption(kLeft2, kLeft2);
    m_chooser.addOption(kLeft2Center, kLeft2Center);
    m_chooser.addOption(kCenter2, kCenter2);
    m_chooser.addOption(kCenter3Left, kCenter3Left);
    m_chooser.addOption(kCenter3Right, kCenter3Right);
    m_chooser.addOption(kRedCenter4, kRedCenter4);
    m_chooser.addOption(kBlueCenter4, kBlueCenter4);
    m_chooser.addOption(kAutoGenerator, kAutoGenerator);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("Auto Delay: ", m_delay);

    SmartDashboard.putNumber("Go X: ", auto_x);
    SmartDashboard.putNumber("Go Y: ", auto_y);
    SmartDashboard.putNumber("Go to Heading: ", auto_heading);

    m_swerve = d;
    m_arm = a;
    needToResetGyro = false;

    autoTimer.reset();
    
}

public void choose()
{
  SmartDashboard.putData("Auto choices", m_chooser);
  m_delay = SmartDashboard.getNumber("Auto Delay: ", m_delay);

  m_delay = Math.min(m_delay, 5.0);
  m_delay = Math.max(m_delay, 0.0);
}

public void selectAuto() {
    m_autoSelected = m_chooser.getSelected();
    m_arm.setState(ArmState.SCORE_SPEAKER);
}

public void runSelectedAuto()
{
  autoTimer.start();
  m_arm.moveCrank();

    switch (m_autoSelected) {
        case kCenter1:
          if(autoTimer.get() < m_delay + 1)
          {
            m_arm.setState(ArmState.SCORE_SPEAKER);
          }
          else if(autoTimer.get() < m_delay + 2)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
          }
          else if(autoTimer.get() < m_delay + 5)
          {
            m_arm.stopAll();
            driveForward(1.3, true);
          }
          else
          {
            m_arm.stopAll();
          }


          break;
        case kCenter2:
          center2();
          break;
        case kRight1:
          scoreAndTurn(60, DrivetrainConstants.COUNTERCLOCKWISE);
          break;
        case kLeft1:
          scoreAndTurn(-60, DrivetrainConstants.CLOCKWISE);
          break;
        case kLeft2:
          scoreAndTurnAndScore(DrivetrainConstants.RIGHT);
          break;
        case kLeft2Center:
          scoreAndTurnAndScore(-60, DrivetrainConstants.RIGHT);
          break;
        case kRight2:
          scoreAndTurnAndScore(DrivetrainConstants.LEFT);
          break;
        case kRight2Center:
          scoreAndTurnAndScore(60, DrivetrainConstants.LEFT);
          break;
        case kRedCenter4:
          redCenter4();
          break;
        case kBlueCenter4:
          blueCenter4();
          break;
        case kAutoGenerator:
          autoGen();
        break;
        case kCenter3Left:
          center3(DrivetrainConstants.LEFT);
          break;
        case kCenter3Right:
          center3(DrivetrainConstants.RIGHT);
          break;
        
        case kDefaultAuto:
        default:
          // Put default auto code here
          break;
    }
}

private void autoGen(){
  auto_x = SmartDashboard.getNumber("Go X: ", auto_x);
  auto_y = SmartDashboard.getNumber("Go Y: ", auto_y);
  auto_heading= SmartDashboard.getNumber("Go to Heading: ", auto_heading);

  driveAngle(auto_x, auto_y, auto_heading, true);
}

private void center2() {
  if(autoTimer.get() < m_delay + .25)
          {
            m_arm.setState(ArmState.SCORE_SPEAKER);
          }
          else if(autoTimer.get() < m_delay + 1)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
          }
          else if(autoTimer.get() < m_delay + 3)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveForward(1.6, false);
          }
          else if(autoTimer.get() < m_delay + 5)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            driveForward(-0.2, false);
            m_arm.setState(ArmState.SCORE_SPEAKER);
          }
          else if(autoTimer.get() < m_delay + 6)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
          }
          else
          {
            m_arm.stopAll();
          }
}

private void center3(int dir) {
  if(autoTimer.get() < m_delay + 0.5)
  {
    ;
  }
  else if(autoTimer.get() < m_delay + 1.3)
  {
    if(m_arm.getCrankPosition() - ArmConstants.SCORE_SPEAKER_ANGLE < 1)
      m_arm.runShooter(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay + 3)
  {
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    m_arm.stopShooter();
    m_arm.runIntake(ArmConstants.FORWARD);
    driveForward(Auto3NoteConstants.AMBI_COLLECT_2, true);
  }
  else if(autoTimer.get() < m_delay + 4.5)
  {
    m_arm.stopIntake();
    m_arm.backUpStager();
    m_arm.setState(ArmState.SCORE_SPEAKER);
    driveForward(Auto3NoteConstants.AMBI_RETURN_2, true);
  }
  else if(autoTimer.get() < m_delay + 5)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay + 7)
  {
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    m_arm.stopShooter();
    m_arm.runIntake(ArmConstants.FORWARD);
    driveAngle(Auto3NoteConstants.AMBI_COLLECT_3, dir*1.8, dir*20, true);
  }
  else if(autoTimer.get() < m_delay + 9)
  {
    m_arm.stopIntake();
    m_arm.backUpStager();
    m_arm.setState(ArmState.SCORE_SPEAKER);
    driveAngle(Auto3NoteConstants.AMBI_RETURN_3, 0, 0, true); //was .3 on practice moved to .6 for elims
  }
  else if(autoTimer.get() < m_delay + 10)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
    m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
  }
  else if(autoTimer.get() < m_delay + 12)
  {
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    m_arm.stopAll();
  }
}

private void blueCenter4() {
if(autoTimer.get() < m_delay + 0.5)
          {
            ;
          }
          else if(autoTimer.get() < m_delay + 1.3)
          {
            if(m_arm.getCrankPosition() - ArmConstants.SCORE_SPEAKER_ANGLE < 1)
              m_arm.runShooter(ArmConstants.FORWARD);
          }
          else if(autoTimer.get() < m_delay + 3)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveForward(Auto4NoteConstants.BLUE_COLLECT_2, true);
          }
          else if(autoTimer.get() < m_delay + 4.5)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveForward(Auto4NoteConstants.BLUE_RETURN_2, true);
          }
          else if(autoTimer.get() < m_delay + 5)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            //m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else if(autoTimer.get() < m_delay + 7)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveAngle(Auto4NoteConstants.BLUE_COLLECT_3, 1.8, 20, true);
          }
          else if(autoTimer.get() < m_delay + 9)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveAngle(Auto4NoteConstants.BLUE_RETURN_3, 0, 0, true); //was .3 on practice moved to .6 for elims
          }
          else if(autoTimer.get() < m_delay + 10)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else if(autoTimer.get() < m_delay + 12)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveAngle(Auto4NoteConstants.BLUE_COLLECT_4, -2.2, -20, true);
          }
          else if(autoTimer.get() < m_delay + 14)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveAngle(Auto4NoteConstants.BLUE_RETURN_4, .2, 0, true); //was .8 on practice
          }
          else if(autoTimer.get() < m_delay + 15)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else
          {
            m_arm.stopAll();
          }



}

private void redCenter4() {
          if(autoTimer.get() < m_delay + 0.5)
          {
            m_arm.setState(ArmState.SCORE_SPEAKER);
          }
          else if(autoTimer.get() < m_delay + 1.3)
          {
            if(m_arm.getCrankPosition() - ArmConstants.SCORE_SPEAKER_ANGLE < 1)
              m_arm.runShooter(ArmConstants.FORWARD);
          }
          else if(autoTimer.get() < m_delay + 3)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveForward(Auto4NoteConstants.RED_COLLECT_2, true);
          }
          else if(autoTimer.get() < m_delay + 4.5)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveForward(Auto4NoteConstants.RED_RETURN_2, true);
          }
          else if(autoTimer.get() < m_delay + 5)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            //m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else if(autoTimer.get() < m_delay + 7)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveAngle(Auto4NoteConstants.RED_COLLECT_3, -1.8, -20, true);
          }
          else if(autoTimer.get() < m_delay + 9)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveAngle(Auto4NoteConstants.RED_RETURN_3, 0, 0, true); //was -.1 before elims
          }
          else if(autoTimer.get() < m_delay + 10)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else if(autoTimer.get() < m_delay + 12)
          {
            m_arm.setState(ArmState.COLLECT_FROM_GROUND);
            m_arm.stopShooter();
            m_arm.runIntake(ArmConstants.FORWARD);
            driveAngle(Auto4NoteConstants.RED_COLLECT_4, 1.9, 20, true);
          }
          else if(autoTimer.get() < m_delay + 14)
          {
            m_arm.stopIntake();
            m_arm.backUpStager();
            m_arm.setState(ArmState.SCORE_SPEAKER);
            driveAngle(Auto4NoteConstants.RED_RETURN_4, -.2, 0, true);
          }
          else if(autoTimer.get() < m_delay + 15)
          {
            m_arm.runShooter(ArmConstants.FORWARD);
            m_swerve.drive(0.0, 0.0, 0.0, 0.0, true, m_period);
          }
          else
          {
            m_arm.stopAll();
          }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    

private void driveForward(double meters, boolean rel){

  double rot = m_swerve.getHeading()* -0.1;
  double speed = autoXDrivePID.calculate(m_swerve.getPose().getX(), meters);

  m_swerve.drive(speed * DrivetrainConstants.AUTO_DRIVE_SPEED, 0, 
                      DrivetrainConstants.AUTO_DRIVE_SPEED, rot, rel, m_period);

}

private void turn(double heading){
  double rot = (m_swerve.getHeading() - heading) * -0.2;
  double speed = 0.0;

  m_swerve.drive(speed * DrivetrainConstants.AUTO_DRIVE_SPEED, 0, 
                      DrivetrainConstants.AUTO_DRIVE_SPEED, rot, true, m_period);
  
}

private void scoreAndTurn(double heading, int direction)
{
  if(autoTimer.get() < m_delay +1)
  {
    m_arm.setState(ArmState.SCORE_SPEAKER);
  }
  else if(autoTimer.get() < m_delay +2)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay +3)
  {
    m_arm.stopAll();
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    driveForward(.5, true);
  }
  else if(autoTimer.get() < m_delay +4)
  {
    turn(heading);
    needToResetGyro = true;
  }
  else if(autoTimer.get() < m_delay +7){

    if(needToResetGyro)
    {
      m_swerve.resetGyroYaw();
      needToResetGyro = false;
    }
  
    driveForward(2, true);
    m_arm.runIntake(ArmConstants.FORWARD);
  }
  else
  {
    m_arm.stopAll();
  }
}

private void scoreAndTurnAndScore(int direction)
{
  if(autoTimer.get() < m_delay + 1)
  {
    m_arm.setState(ArmState.SCORE_SPEAKER);
  }
  else if(autoTimer.get() < m_delay + 2)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay + 4)
  {
    m_arm.stopShooter();
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    m_arm.runIntake(ArmConstants.FORWARD);
    driveAngle(1.5, direction*2, direction*45, true);
  }
  else if(autoTimer.get() < m_delay + 6)
  {
    m_arm.stopIntake();
    m_arm.backUpStager();
    m_arm.setState(ArmState.SCORE_SPEAKER);
    driveAngle(-.35, -direction*.25, 0, true);
  }
  else if(autoTimer.get() < m_delay + 8)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
    m_swerve.drive(0,0,0,0, false, m_period);
  }
  else
  {
    m_arm.stopAll();
  }
}

private void scoreAndTurnAndScore(double heading, int direction)
{
  if(autoTimer.get() < m_delay + 1)
  {
    m_arm.setState(ArmState.SCORE_SPEAKER);
  }
  else if(autoTimer.get() < m_delay + 2)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay + 3)
  {
    m_arm.stopAll();
    m_arm.setState(ArmState.COLLECT_FROM_GROUND);
    driveForward(.5, true);
  }
  else if(autoTimer.get() < m_delay + 5)
  {
     turn(heading);
     needToResetGyro = true;
  }
  else if(autoTimer.get() < m_delay + 7)
  {
    if(needToResetGyro)
    {
      m_swerve.resetGyroYaw();
      m_swerve.resetPosition();
      needToResetGyro = false;
    }
    driveForward(2.5, true);
    m_arm.runIntake(ArmConstants.FORWARD);
  }
  else if(autoTimer.get() < m_delay + 9)
  {
    m_arm.stopIntake();
    m_arm.backUpStager();
    driveAngle(.5, direction*2, 0, true);
    m_arm.setState(ArmState.SCORE_SPEAKER);
  }
  else if(autoTimer.get() < m_delay + 11)
  {
    m_arm.runShooter(ArmConstants.FORWARD);
    m_swerve.drive(0,0,0,0, false, m_period);
  }
  else
  {
    m_arm.stopAll();
  }
}

public void driveAngle(double x_meters, double y_meters, double desiredHeading, boolean rel){
  double x = m_swerve.getPose().getX();
  double y = m_swerve.getPose().getY();
  double ratio = 1;
  double x_diff = x - x_meters;
  double y_diff = y - y_meters;

  if(x_diff != 0 && y_diff != 0)
    ratio = Math.abs((y_diff)/(x_diff));

  double rot = (m_swerve.getHeading() - desiredHeading) * -0.1;
  double x_speed = autoXDrivePID.calculate(x, x_meters);
  double y_speed = autoYDrivePID.calculate(y, y_meters);

  //m_swerve.drive(m_xAutoSlew.calculate(speed) * DrivetrainConstants.AUTO_DRIVE_SPEED, 0, 
                      //DrivetrainConstants.AUTO_DRIVE_SPEED, rot, rel, m_period);


  double x_ff= 0; //Math.signum(x_speed)*.02; //x_driveFeedforward.calculate(x_speed * DrivetrainConstants.AUTO_DRIVE_SPEED * 1.0 / ratio);
  double y_ff= 0; //Math.signum(y_speed)*.02;//y_driveFeedforward.calculate(y_speed * DrivetrainConstants.AUTO_DRIVE_SPEED * ratio);

  m_swerve.drive(
    x_speed * DrivetrainConstants.AUTO_DRIVE_SPEED * 1.0 / ratio + x_ff, 
    y_speed * DrivetrainConstants.AUTO_DRIVE_SPEED * ratio + y_ff, 
                      DrivetrainConstants.AUTO_DRIVE_SPEED * 2, 
                      MathUtil.applyDeadband(rot, 0.1)
                      , rel, m_period);

  
  /*  if(x_direction > 0){
    if(meters > m_swerve.getPose().getX()){
      m_swerve.drive(m_xAutoSlew.calculate(Math.cos(test_angle)) *speed * x_direction * DrivetrainConstants.AUTO_DRIVE_SPEED, 
                     m_yAutoSlew.calculate(Math.sin(test_angle)) *speed * y_direction * DrivetrainConstants.AUTO_DRIVE_SPEED, 
                      DrivetrainConstants.AUTO_DRIVE_SPEED, rot, rel, m_period);
    } else {
      m_swerve.drive(0, 0, 3.0, 0, false, m_period);
    } 
  } else {
    if(meters < m_swerve.getPose().getX()){
      m_swerve.drive(m_xAutoSlew.calculate(x_direction) * DrivetrainConstants.AUTO_DRIVE_SPEED,
                     m_yAutoSlew.calculate(y_direction) * DrivetrainConstants.AUTO_DRIVE_SPEED,
                      DrivetrainConstants.AUTO_DRIVE_SPEED, rot, rel, m_period);
    } else {
      m_swerve.drive(0, 0, 3.0, 0, false, m_period);
    }
  }

  **/

}

public void resetTimer()
{
  autoTimer.reset();
}

public void setPeriod(double p)
{
  m_period = p;
}

    
}
