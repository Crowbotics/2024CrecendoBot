package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PID;

public class Arm {

    //Crank PID Constants
double kMaxOutput = .7, kMinOutput= -.7;

private final CANSparkMax m_intake = new CANSparkMax(11, MotorType.kBrushless);//good
private final CANSparkMax m_LeftShooter = new CANSparkMax(10, MotorType.kBrushless); //good
private final CANSparkMax m_RightShooter = new CANSparkMax(13, MotorType.kBrushless); //good
private final CANSparkMax m_stager = new CANSparkMax(12, MotorType.kBrushless); //good
private final CANSparkMax m_crank = new CANSparkMax(15, MotorType.kBrushless); //good
private final AbsoluteEncoder m_crankEncoder;

private final Servo m_climbLock = new Servo(1);

private final DigitalInput m_noteSensor = new DigitalInput(0);

private double crankPosition;
private double intakeSpeed;
private double shootSpeed;
private double stagerSpeed;


private SparkPIDController m_crankPID = m_crank.getPIDController();
private SparkPIDController m_stagerPID = m_stager.getPIDController();

public ArmState m_ArmState;

private boolean noteVisiblilityChange = false;
private boolean noteVisibleNow = false;
private boolean noteVisibleBefore = false;
private boolean needToBackup = false;



public static enum ArmState{
        DEFAULT, COLLECT_FROM_GROUND, COLLECT_FROM_SOURCE, SCORE_SPEAKER, SCORE_AMP
}

public Arm()
{
    m_crankEncoder =  m_crank.getAbsoluteEncoder();
    Timer.delay(0.02);

    m_LeftShooter.setInverted(true);
    m_RightShooter.setInverted(true);
    m_stager.setInverted(true);
    m_intake.setInverted(true);;

    //m_crankEncoder.setAverageDepth(8);
    m_crankEncoder.setPositionConversionFactor(360.0);
    m_crankPID.setFeedbackDevice(m_crankEncoder);
    m_crankPID.setP(PID.kP_arm);
    m_crankPID.setI(PID.kI_arm);
    m_crankPID.setD(PID.kD_arm);
    m_crankPID.setOutputRange(kMinOutput, kMaxOutput);

    m_stagerPID.setP(.01);

    setState(ArmState.DEFAULT);
}

public void setState(ArmState st)
{
    m_ArmState = st;

    resetFlags();

    switch(m_ArmState)
    {
        case COLLECT_FROM_SOURCE:
        crankPosition = Constants.ArmConstants.COLLECT_SOURCE_ANGLE;
        intakeSpeed = -0.2;
        shootSpeed = 0;
        stagerSpeed = -0.3;
        break;

        case COLLECT_FROM_GROUND:
        crankPosition = Constants.ArmConstants.COLLECT_GROUND_ANGLE;
        intakeSpeed = 1;
        shootSpeed = 0.9;
        stagerSpeed = 0.5;
        break;

        case SCORE_AMP:
        crankPosition = Constants.ArmConstants.SCORE_AMP_ANGLE;
        intakeSpeed = 1;
        shootSpeed = Constants.ArmConstants.AMP_SCORE_SPEED;
        stagerSpeed = 0.5;
        break;

        case SCORE_SPEAKER:
        crankPosition = Constants.ArmConstants.SCORE_SPEAKER_ANGLE;
        shootSpeed = Constants.ArmConstants.SPEAKER_SCORE_SPEED;
        stagerSpeed = 0.5;
        intakeSpeed = 0.5;
        break;

        default:
        crankPosition = 250;
        intakeSpeed = 0;
        shootSpeed = 0;
    }

}

public boolean hasNote()
{
    return m_noteSensor.get();
}

public void moveCrank()
{
    m_crankPID.setReference(crankPosition, CANSparkMax.ControlType.kPosition);
}

public void runIntake(int direction)
{
    if(direction == ArmConstants.REVERSE)
        resetFlags();

    if(needToBackup)
    {
        backUpStager();
    }
    else
    {

        if(m_ArmState.equals(ArmState.COLLECT_FROM_GROUND) || 
            (m_ArmState.equals(ArmState.SCORE_SPEAKER) && direction == 1))
            {
            m_intake.set(direction * intakeSpeed);
            }

        if(m_ArmState.equals(ArmState.COLLECT_FROM_SOURCE))
        {
            m_LeftShooter.set(direction * intakeSpeed);
            m_RightShooter.set(direction * intakeSpeed);
        }

        
        if(intakeTooFar() &&                                 //Did we blow past laser?
            direction == Constants.ArmConstants.FORWARD &&          //Are we collecting
            m_ArmState.equals(ArmState.COLLECT_FROM_GROUND)) {      //In the GROUND state
    
                needToBackup = true;
                noteVisiblilityChange = false;
                stopIntake();
        }
        else
        {
            m_stager.set(direction * stagerSpeed);
        }
    }

}

public void runShooter(int direction)
{
    needToBackup = false;
    noteVisiblilityChange = false;
    
    m_LeftShooter.set(direction * shootSpeed);
    m_RightShooter.set(direction * shootSpeed);

    if((m_ArmState.equals(ArmState.SCORE_SPEAKER) || m_ArmState.equals(ArmState.COLLECT_FROM_GROUND))
        && 
        Math.abs(m_LeftShooter.getEncoder().getVelocity()) > 2000 
        ||
        m_ArmState.equals(ArmState.SCORE_AMP)){
            m_intake.set(.2);
            m_stager.set(direction * .3);

        }else{
            m_stager.set(-0.1);
        }
    

}

public void stopAll()
{
    stopShooter();
    stopIntake();
    stopStager();
}

public void stopShooter()
{
    m_LeftShooter.set(0);
    m_RightShooter.set(0);
}

public void stopIntake()
{
    m_intake.set(0);
}

public void stopStager()
{
    m_stager.set(0);
}

public String getState()
{
    return m_ArmState.toString();
}

public double getCrankPosition()
{
    return m_crankEncoder.getPosition();
}

public double getDesiredCrankPosition()
{
    return crankPosition;
}

public void backUpStager()
{
    if(hasNote())
    {
        needToBackup = false;
    }
    else
    {
        m_stager.set(-0.1);
        m_LeftShooter.set(-0.1);
        m_RightShooter.set(-0.1);
        m_intake.set(.3);
    }
}

public boolean needToBackup()
{
    return needToBackup;
}

public void Purge(double direction)
{
    resetFlags();

    //If we want to purge forward, wait until shooter spins up
    if(direction == ArmConstants.FORWARD)
    {
        crankPosition = 263;
        if(Math.abs(m_LeftShooter.getEncoder().getVelocity()) < 2000
            || (Math.abs(getCrankPosition() - 263) > 4))
            m_stager.set(0);
        else
            m_stager.set(direction);
    }
    else{
        m_stager.set(direction);
    }

    m_LeftShooter.set(direction);
    m_RightShooter.set(direction);
    m_intake.set(direction);
}

public boolean intakeTooFar()
{
    noteVisibleBefore = noteVisibleNow;
    noteVisibleNow = hasNote();

    //Did we blow past the sensor?
    if(noteVisibleBefore  == true &&  noteVisibleNow == false)
    {
        noteVisiblilityChange = true;
    }

    return noteVisiblilityChange;
}

public boolean shouldRumble()
{
    if((m_ArmState.equals(ArmState.COLLECT_FROM_GROUND) && hasNote())
    || needToBackup == true)
        return true;
    else
        return false;
}

public void resetFlags()
{
    needToBackup = false;
    noteVisiblilityChange = false;
    noteVisibleNow = hasNote();
    noteVisibleBefore = noteVisibleNow;
}

public void setClimbLock(int i) {
    if(i == 0)
    {
        crankPosition = ArmConstants.CLIMB_ANGLE;

        if(Math.abs(getCrankPosition() - ArmConstants.CLIMB_ANGLE ) < 1)
            m_climbLock.set(0);

    } else {
        m_climbLock.set(1);
    }   
}

public void updatePIDValues() {
    m_crankPID.setP(PID.kP_arm);
    m_crankPID.setI(PID.kI_arm);
    m_crankPID.setD(PID.kD_arm);
}

}
