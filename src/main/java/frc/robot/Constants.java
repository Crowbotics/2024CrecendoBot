package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {

    public static final class Auto4NoteConstants {
        // ~~~*~~~ BLUE 4 NOTE DISTANCES ~~~*~~~
        //Sets the offsets for our 2nd, 3rd and 4th notes
        //Tuned 4/19/2024 for Worlds
        public static final double BLUE_COLLECT_2 = 1.6;
        public static final double BLUE_RETURN_2 = -0.2;
        public static final double BLUE_COLLECT_3 = 1.8; //Mised w/ 1.6 on Match 10
        public static final double BLUE_RETURN_3 = -0.3;
        public static final double BLUE_COLLECT_4 = 1.6; //Left Alone
        public static final double BLUE_RETURN_4 = -0.6;

        // ~~~*~~~ RED 4 NOTE DISTANCES ~~~*~~~
        //Sets the offsets for our 2nd, 3rd and 4th notes
        //Tuned 4/19/2024 for Worlds
        public static final double RED_COLLECT_2 = 1.6;
        public static final double RED_RETURN_2 = 0;
        public static final double RED_COLLECT_3 = 1.7; //Was 1.7 and worked even though 1.8 is our blue #.
        public static final double RED_RETURN_3 = -0.4;
        public static final double RED_COLLECT_4 = 1.7;
        public static final double RED_RETURN_4 = -0.6; 
    }

    public static final class Auto3NoteConstants{
        //Used for 3 note. Worked on both sides.
        //Tuned 4/19/2024 for Worlds
        public static final double AMBI_COLLECT_2 = 1.6;
        public static final double AMBI_RETURN_2 = -0.2;
        public static final double AMBI_COLLECT_3 = 1.6;
        public static final double AMBI_RETURN_3 = -0.4;
    }

    public static final class PID {
        //Values tuned at CIR
        public static double kP_drive = 2.25;
        public static double kI_drive = 0;
        public static double kD_drive = 0.01;

        public static double kP_turn = 4.5;
        public static double kI_turn = 0;
        public static double kD_turn = 0;

        public static double kP_arm = 0.1;
        public static double kI_arm = 0;
        public static double kD_arm = 1;

    }

    public static final class ArmConstants {
        public static final int FORWARD = 1;
        public static final int REVERSE = -1; 

        //Original Arm Level: 105
        //NEW Arm Level: 

        public static final double CLIMB_ANGLE = 100;                
        public static final double SCORE_AMP_ANGLE = 255;           
        public static final double SCORE_SPEAKER_ANGLE = 164;       
        public static final double COLLECT_GROUND_ANGLE = 108; //Was 117
        public static final double COLLECT_SOURCE_ANGLE = 252;

        public static final double AMP_SCORE_SPEED = .35;
        public static final double SPEAKER_SCORE_SPEED = 1;
    }

    public static final class DrivetrainConstants {
        public static final double MAX_DRIVE_SPEED = 16.0; // 10 meters per second
        public static final double SLOW_DRVE_SPEED = 3.0;
        public static final double AUTO_DRIVE_SPEED = 6.0;
        public static final double MAX_ANGLE_SPEED = 4.5*Math.PI; // 1/2 rotation per second

        public static final int FORWARD = 1;
        public static final int REVERSE = -1; 

        public static final int LEFT = 1;
        public static final int RIGHT = -1;

        public static final int COUNTERCLOCKWISE = 1;
        public static final int CLOCKWISE = -1;
    }

    public static final class ModuleConstants {
        public static final double kWheelRadius = 0.0508;
        public static final double kWheelDiameterMeters = 0.10033;
        public static final double kDriveMotorGearRatio = 5.96;
        public static final double kTurningMotorGearRatio = 1 /12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    }

    public static void updatePID()
    {
        PID.kP_drive = SmartDashboard.getNumber("Drive P: ", PID.kP_drive);
        PID.kI_drive = SmartDashboard.getNumber("Drive I: ", PID.kI_drive);
        PID.kD_drive = SmartDashboard.getNumber("Drive D: ", PID.kD_drive);

        PID.kP_turn = SmartDashboard.getNumber("Turn P: ", PID.kP_turn);
        PID.kI_turn = SmartDashboard.getNumber("Turn I: ", PID.kI_turn);
        PID.kD_turn = SmartDashboard.getNumber("Turn D: ", PID.kD_turn);

        PID.kP_arm = SmartDashboard.getNumber("Arm P: ", PID.kP_arm);
        PID.kI_arm = SmartDashboard.getNumber("Arm I: ", PID.kI_arm);
        PID.kD_arm = SmartDashboard.getNumber("Arm D: ", PID.kD_arm);
    }

    public static void addPIFtoSmartDashboard() {
        SmartDashboard.putNumber("Drive P: ", PID.kP_drive);
        SmartDashboard.putNumber("Drive I: ", PID.kI_drive);
        SmartDashboard.putNumber("Drive D: ", PID.kD_drive);

        SmartDashboard.putNumber("Turn P: ", PID.kP_turn);
        SmartDashboard.putNumber("Turn I: ", PID.kI_turn);
        SmartDashboard.putNumber("Turn D: ", PID.kD_turn);

        SmartDashboard.putNumber("Arm P: ", PID.kP_arm);
        SmartDashboard.putNumber("Arm I: ", PID.kI_arm);
        SmartDashboard.putNumber("Arm D: ", PID.kD_arm);
    }
}