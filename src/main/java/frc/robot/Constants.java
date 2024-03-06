package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Lights.RGBColors;

public final class Constants {
    
    public static final class ModuleConstants {
        //The diameter of the wheel on your swerve drive
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94); 
        //The gear ratio for an SDS MK4 module with speed ratio of L2
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        //Ratio for an SDS MK4 turning motor
        public static final double kTurningMotorGearRatio = 1 / 12.80;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double upDOG = 36.0/36;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        //The P value for the turning PID loop
        public static final double kPTurning = 0.2;
    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int gamepadPort = 2;

        public static final int btn_a = 1;
        public static final int btn_b = 2;
        public static final int btn_x = 3;
        public static final int btn_y = 4;
        public static final int btn_leftBumper = 5;
        public static final int btn_rightBumper = 6;
        public static final int btn_select = 7;
        public static final int btn_start = 8;

        // Axises
        public static final int rightTriggerAxis = 3;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(22);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(22);


        // Measured from the center of the robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),    /*FL*/
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   /*FR*/
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   /*BL*/
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /*BR*/

        //CAN IDs For the drive motor spark maxes on the swerve modules
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackLeftDriveMotorPort = 30;
        public static final int kBackRightDriveMotorPort = 40;

        //CAN IDs for the turning motor spark maxes on the swerve modules
        public static final int kFrontLeftTurningMotorPort = 15;
        public static final int kFrontRightTurningMotorPort = 25;
        public static final int kBackLeftTurningMotorPort = 35;
        public static final int kBackRightTurningMotorPort = 45;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values muest result in a counter-clockwise movment
        //Note: if you are using the Mk4i (Inverted) Swerve Modules, you may need to change this to "true"
        //Hint: if there is a big oscillation when you turn, try inverting these.
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values must result in a forward movement
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        //The CAN id's for the CANcoder's on the swerve modules
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 32;
        public static final int kBackRightDriveAbsoluteEncoderPort = 42;

        //Inversion of the direction of the CANcoder
        //Positive values must result in a counter-clockwise movement
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //Position must be counter-clockwise from the positive YAw
        public static final boolean kGyroInverted = false;

        //The offset of the CANcoder's position from the zero position (Straight forward)
        //Measure this by rotating all the modules to the forward position and reading the CANcoder's value
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(241.3);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(283.7);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(131.4);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(196.4);

        // in m/s, based on MK4 L2 speed of 14.5 ft/s
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);  
        // Robot turning speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5 * Math.PI;

        // If you want to slow down the robot during TeleOp, adjust these values
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;  // Slowed down for testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;





    



    }

    

    public static final class SubsystemConstants {
        //Arm Subsystem
        public static final int ArmLeftCANid = 55;
        public static final int ArmRIghtCANid = 56;

        //Intake subsystem
        public static final int IntakeCANid = 50;
        
        //Shooter Subsystem
        public static final int ShooterTopCANid = 51;
        public static final int ShooterBottomCANid = 52;
        public static final int ShooterMAXSpeedRMPs = 5000;

        //Climb Subsystem
        public static final int ClimbLeftCANid = 53;
        public static final int ClimbRightCANid = 54;

        //LED Subsystem
        public static final int CANdleID = 0;
    }

    public static final class CommandConstants {
        //Climb Commands
        // public static final int lowestClimbCmd = 1;
        // public static final int middleClimbCmd = 2;
        // public static final int highestClimbCmd = 3;

        public static final class Climb {
            public static final double highPos = 100.0;
            public static final double midPos = 50.0;
            public static final double windSpeed = -1;
            public static final double unwindSpeed = -windSpeed;
        }
        //Arm Commands
        public static final class Arm {
            public static final double AbsEncoderOffset=.77;
            public static final double amp = 100.0;
            public static final double intake = -7;
            public static final double traversal = 60;
            public static final double closeSpeaker = 12;
            // Arm Feedforward Constants
            public static final double kP = 1;
            // Values Obtained from Characterization via reca.lc/arm

            // 10lbs Config
            // public static final double kSVolts = 1;
            // public static final double kGVolts = .45;
            // public static final double kVVoltSecondPerRad = 2.49;
            // public static final double kAVoltSecondSquaredPerRad = 0.03;

            // 20lbs Config
            public static final double kSVolts = 1;
            public static final double kGVolts = 0.90;
            public static final double kVVoltSecondPerRad = 2.49;
            public static final double kAVoltSecondSquaredPerRad = 0.03;

            // OTHER CONSTANTS - Unkown Values
            public static final double kMaxVelocityRadPerSecond = 3;
            public static final double kMaxAccelerationRadPerSecSquared = 10;
        }
        
    }

    public static final class AprilTagPositions {
        public static final double Tag4_x = 8.308;
        public static final double Tag4_y = -2.722;
    }

    public static final class AutoConstants {
        // If you want to slow down the robot during Autonomous, adjust these values
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond /10; 
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxRotationalVelocityMetersPerSecond = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        // The P value of the PID controller used in auto for the X and Y directions
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        // The P value of the PID controller used in auto for the theta (rotation) direction
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class TrajectoryConstants {
      
        // Constant position in meters for all game pieces
        // Note position closest to each driver station wall.
        // ----------- Blue Alliance -----------
        // Object Structure: X (In Meters), Y (In Meters), Angle (In Degrees)
        public static final class blue {
            public static final double[] stageNote = {-5.540121, 0, 0};
            public static final double[] speakerNote = {-5.540121, 1.4478, 0};
            public static final double[] ampNote = {-5.540121, 2.8956, 0};
        }
        // ----------- Red Alliance -----------
        // Object Structure: X (In Meters), Y (In Meters), Angle (In Degrees)
        // Add 180 to the angle for the red alliance as robot intake needs to face center of field
        public static final class red {
            public static final double[] stageNote = {5.616321, 0, 180};
            public static final double[] speakerNote = {5.616321, 1.4478, 180};
            public static final double[] ampNote = {5.616321, 2.8956, 180};
        }
    }


    public static final class OIConstants {
        // Port for the driver's controller (Thrust Master)
        public static final int kDriverControllerPort = 0;
        // Axis used for the X, Y, Rotation, and Throttle
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 3;
        public static final int kDriverThrottleAxis = 2;
        // Deadband for the controller
        public static final double kDeadband = 0.05;
        // Button used to enable robot orientation driving
        public static final int kDriverFieldOrientedButtonIdx = 5;
        // Button used to enable slow turning
        public static final int kDriverSlowTurnButtonIdx=1;
        // Button used to reset the gyro to 0
        public static final int kDriverResetGyroButtonIdx=2;


    }

      public static class LightsConstants {
        public static final int candlePort = 26;
    }

    public static class LightDesign {
        public final static int[][][] WIRED_WIZARDS = {
            // =(CONCATENATE("Test",HEX2DEC(MID(myBackgroundRanges("B2",B2),2,2))))
            // BLACK SURROUNDING Wiz Logo
            { { 0, 6 }, RGBColors.BLACK },
            { { 0, 9 }, RGBColors.BLACK },
            { { 1, 5 }, RGBColors.BLACK },
            { { 1, 8 }, RGBColors.BLACK },
            { { 2, 4 }, RGBColors.BLACK },
            { { 2, 9 }, RGBColors.BLACK },
            { { 3, 6 }, RGBColors.BLACK },
            { { 3, 8 }, RGBColors.BLACK },
            { { 4, 5 }, RGBColors.BLACK },
            { { 4, 7 }, RGBColors.BLACK },
            { { 5, 4 }, RGBColors.BLACK },
            { { 5, 8 }, RGBColors.BLACK },
            { { 6, 5 }, RGBColors.BLACK },
            { { 6, 8 }, RGBColors.BLACK },
            { { 7, 5 }, RGBColors.BLACK },
            { { 7, 7 }, RGBColors.BLACK },
            // BLACK SURRONING NUMS
            { { 1, 11 }, RGBColors.BLACK },
            { { 1, 13 }, RGBColors.BLACK },
            { { 1, 15 }, RGBColors.BLACK },
            { { 1, 19 }, RGBColors.BLACK },
            { { 1, 23 }, RGBColors.BLACK },
            { { 1, 25 }, RGBColors.BLACK },
            { { 1, 27 }, RGBColors.BLACK },
            { { 2, 11 }, RGBColors.BLACK },
            { { 2, 13 }, RGBColors.BLACK },
            { { 2, 15 }, RGBColors.BLACK },
            { { 2, 17 }, RGBColors.BLACK },
            { { 2, 21 }, RGBColors.BLACK },
            { { 2, 23 }, RGBColors.BLACK },
            { { 2, 25 }, RGBColors.BLACK },
            { { 2, 27 }, RGBColors.BLACK },
            { { 3, 11 }, RGBColors.BLACK },
            { { 3, 13 }, RGBColors.BLACK },
            { { 3, 15 }, RGBColors.BLACK },
            { { 3, 17 }, RGBColors.BLACK },
            { { 3, 21 }, RGBColors.BLACK },
            { { 3, 23 }, RGBColors.BLACK },
            { { 3, 25 }, RGBColors.BLACK },
            { { 3, 27 }, RGBColors.BLACK },
            { { 4, 11 }, RGBColors.BLACK },
            { { 4, 15 }, RGBColors.BLACK },
            { { 4, 19 }, RGBColors.BLACK },
            { { 4, 23 }, RGBColors.BLACK },
            { { 4, 27 }, RGBColors.BLACK },
            { { 5, 13 }, RGBColors.BLACK },
            { { 5, 15 }, RGBColors.BLACK },
            { { 5, 17 }, RGBColors.BLACK },
            { { 5, 21 }, RGBColors.BLACK },
            { { 5, 23 }, RGBColors.BLACK },
            { { 5, 25 }, RGBColors.BLACK },
            { { 5, 27 }, RGBColors.BLACK },
            { { 6, 13 }, RGBColors.BLACK },
            { { 6, 15 }, RGBColors.BLACK },
            { { 6, 17 }, RGBColors.BLACK },
            { { 6, 19 }, RGBColors.BLACK },
            { { 6, 21 }, RGBColors.BLACK },
            { { 6, 23 }, RGBColors.BLACK },
            { { 6, 25 }, RGBColors.BLACK },
            { { 6, 27 }, RGBColors.BLACK },
            { { 7, 13 }, RGBColors.BLACK },
            { { 7, 15 }, RGBColors.BLACK },
            { { 7, 19 }, RGBColors.BLACK },
            { { 7, 23 }, RGBColors.BLACK },
            { { 7, 25 }, RGBColors.BLACK },
            { { 7, 27 }, RGBColors.BLACK },

            // Wiz Logo
            { { 0, 7 }, RGBColors.DARKGRAY },
            { { 0, 8 }, RGBColors.DARKGRAY },
            { { 1, 6 }, RGBColors.DARKGRAY },
            { { 1, 7 }, RGBColors.DARKGRAY },
            { { 2, 5 }, RGBColors.DARKGRAY },
            { { 2, 6 }, RGBColors.DARKGRAY },
            { { 2, 7 }, RGBColors.DARKGRAY },
            { { 2, 8 }, RGBColors.DARKGRAY },
            { { 3, 7 }, RGBColors.RED },
            { { 4, 6 }, RGBColors.LIGHTGRAY },
            { { 5, 5 }, RGBColors.LIGHTGRAY },
            { { 5, 6 }, RGBColors.LIGHTGRAY },
            { { 5, 7 }, RGBColors.LIGHTGRAY },
            { { 6, 6 }, RGBColors.LIGHTGRAY },
            { { 6, 7 }, RGBColors.LIGHTGRAY },
            { { 7, 6 }, RGBColors.LIGHTGRAY },
            // 4534 Text
            { { 1, 12 }, RGBColors.RED },
            { { 1, 14 }, RGBColors.RED },
            { { 1, 16 }, RGBColors.RED },
            { { 1, 17 }, RGBColors.RED },
            { { 1, 18 }, RGBColors.RED },
            { { 1, 20 }, RGBColors.RED },
            { { 1, 21 }, RGBColors.RED },
            { { 1, 22 }, RGBColors.RED },
            { { 1, 24 }, RGBColors.RED },
            { { 1, 26 }, RGBColors.RED },
            { { 2, 12 }, RGBColors.RED },
            { { 2, 14 }, RGBColors.RED },
            { { 2, 16 }, RGBColors.RED },
            { { 2, 22 }, RGBColors.RED },
            { { 2, 24 }, RGBColors.RED },
            { { 2, 26 }, RGBColors.RED },
            { { 3, 12 }, RGBColors.RED },
            { { 3, 14 }, RGBColors.RED },
            { { 3, 16 }, RGBColors.RED },
            { { 3, 22 }, RGBColors.RED },
            { { 3, 24 }, RGBColors.RED },
            { { 3, 26 }, RGBColors.RED },
            { { 4, 12 }, RGBColors.RED },
            { { 4, 13 }, RGBColors.RED },
            { { 4, 14 }, RGBColors.RED },
            { { 4, 16 }, RGBColors.RED },
            { { 4, 17 }, RGBColors.RED },
            { { 4, 18 }, RGBColors.RED },
            { { 4, 20 }, RGBColors.RED },
            { { 4, 21 }, RGBColors.RED },
            { { 4, 22 }, RGBColors.RED },
            { { 4, 24 }, RGBColors.RED },
            { { 4, 25 }, RGBColors.RED },
            { { 4, 26 }, RGBColors.RED },
            { { 5, 14 }, RGBColors.RED },
            { { 5, 18 }, RGBColors.RED },
            { { 5, 22 }, RGBColors.RED },
            { { 5, 26 }, RGBColors.RED },
            { { 6, 14 }, RGBColors.RED },
            { { 6, 18 }, RGBColors.RED },
            { { 6, 22 }, RGBColors.RED },
            { { 6, 26 }, RGBColors.RED },
            { { 7, 14 }, RGBColors.RED },
            { { 7, 16 }, RGBColors.RED },
            { { 7, 17 }, RGBColors.RED },
            { { 7, 18 }, RGBColors.RED },
            { { 7, 20 }, RGBColors.RED },
            { { 7, 21 }, RGBColors.RED },
            { { 7, 22 }, RGBColors.RED },
            { { 7, 26 }, RGBColors.RED },
        };
        public final static int[][][] nCino = {
        {{1,7},{0,0,1,100}},
        {{1,8},{207,16,19,100}},
        {{1,9},{0,0,1,100}},
        {{1,17},{0,0,1,100}},
        {{1,18},{102,102,102,100}},
        {{1,19},{0,0,1,100}},
        {{2,7},{0,0,1,100}},
        {{2,8},{207,16,19,100}},
        {{2,9},{0,0,1,100}},
        {{2,10},{0,0,1,100}},
        {{2,11},{102,102,102,100}},
        {{2,12},{102,102,102,100}},
        {{2,13},{102,102,102,100}},
        {{2,14},{0,0,1,100}},
        {{2,15},{102,102,102,100}},
        {{2,16},{102,102,102,100}},
        {{2,17},{0,0,1,100}},
        {{2,19},{0,0,1,100}},
        {{2,20},{102,102,102,100}},
        {{2,21},{102,102,102,100}},
        {{2,22},{102,102,102,100}},
        {{2,23},{0,0,1,100}},
        {{2,24},{102,102,102,100}},
        {{2,25},{102,102,102,100}},
        {{2,26},{102,102,102,100}},
        {{2,27},{0,0,1,100}},
        {{3,5},{0,0,1,100}},
        {{3,6},{92,182,77,100}},
        {{3,7},{0,0,1,100}},
        {{3,8},{207,16,19,100}},
        {{3,9},{0,0,1,100}},
        {{3,10},{0,0,1,100}},
        {{3,11},{102,102,102,100}},
        {{3,12},{0,0,1,100}},
        {{3,13},{102,102,102,100}},
        {{3,14},{0,0,1,100}},
        {{3,15},{102,102,102,100}},
        {{3,16},{0,0,1,100}},
        {{3,17},{0,0,1,100}},
        {{3,18},{102,102,102,100}},
        {{3,19},{0,0,1,100}},
        {{3,20},{102,102,102,100}},
        {{3,21},{0,0,1,100}},
        {{3,22},{102,102,102,100}},
        {{3,23},{0,0,1,100}},
        {{3,24},{102,102,102,100}},
        {{3,25},{0,0,1,100}},
        {{3,26},{102,102,102,100}},
        {{3,27},{0,0,1,100}},
        {{4,5},{0,0,1,100}},
        {{4,6},{92,182,77,100}},
        {{4,7},{253,188,1,100}},
        {{4,8},{207,16,19,100}},
        {{4,9},{0,0,1,100}},
        {{4,10},{0,0,1,100}},
        {{4,11},{102,102,102,100}},
        {{4,12},{0,0,1,100}},
        {{4,13},{102,102,102,100}},
        {{4,14},{0,0,1,100}},
        {{4,15},{102,102,102,100}},
        {{4,16},{0,0,1,100}},
        {{4,17},{0,0,1,100}},
        {{4,18},{102,102,102,100}},
        {{4,19},{0,0,1,100}},
        {{4,20},{102,102,102,100}},
        {{4,21},{0,0,1,100}},
        {{4,22},{102,102,102,100}},
        {{4,23},{0,0,1,100}},
        {{4,24},{102,102,102,100}},
        {{4,25},{0,0,1,100}},
        {{4,26},{102,102,102,100}},
        {{4,27},{0,0,1,100}},
        {{5,4},{0,0,1,100}},
        {{5,5},{23,170,220,100}},
        {{5,6},{92,182,77,100}},
        {{5,7},{253,188,1,100}},
        {{5,8},{207,16,19,100}},
        {{5,9},{0,0,1,100}},
        {{5,10},{0,0,1,100}},
        {{5,11},{102,102,102,100}},
        {{5,12},{0,0,1,100}},
        {{5,13},{102,102,102,100}},
        {{5,14},{0,0,1,100}},
        {{5,15},{102,102,102,100}},
        {{5,16},{0,0,1,100}},
        {{5,17},{0,0,1,100}},
        {{5,18},{102,102,102,100}},
        {{5,19},{0,0,1,100}},
        {{5,20},{102,102,102,100}},
        {{5,21},{0,0,1,100}},
        {{5,22},{102,102,102,100}},
        {{5,23},{0,0,1,100}},
        {{5,24},{102,102,102,100}},
        {{5,25},{0,0,1,100}},
        {{5,26},{102,102,102,100}},
        {{5,27},{0,0,1,100}},
        {{6,4},{0,0,1,100}},
        {{6,5},{23,170,220,100}},
        {{6,6},{92,182,77,100}},
        {{6,7},{253,188,1,100}},
        {{6,8},{207,16,19,100}},
        {{6,9},{0,0,1,100}},
        {{6,10},{0,0,1,100}},
        {{6,11},{102,102,102,100}},
        {{6,12},{0,0,1,100}},
        {{6,13},{102,102,102,100}},
        {{6,14},{0,0,1,100}},
        {{6,15},{102,102,102,100}},
        {{6,16},{0,0,1,100}},
        {{6,17},{0,0,1,100}},
        {{6,18},{102,102,102,100}},
        {{6,19},{0,0,1,100}},
        {{6,20},{102,102,102,100}},
        {{6,21},{0,0,1,100}},
        {{6,22},{102,102,102,100}},
        {{6,23},{0,0,1,100}},
        {{6,24},{102,102,102,100}},
        {{6,25},{0,0,1,100}},
        {{6,26},{102,102,102,100}},
        {{6,27},{0,0,1,100}},
        {{7,4},{0,0,1,100}},
        {{7,5},{23,170,220,100}},
        {{7,6},{92,182,77,100}},
        {{7,7},{253,188,1,100}},
        {{7,8},{207,16,19,100}},
        {{7,9},{0,0,1,100}},
        {{7,10},{0,0,1,100}},
        {{7,11},{102,102,102,100}},
        {{7,12},{0,0,1,100}},
        {{7,13},{102,102,102,100}},
        {{7,14},{0,0,1,100}},
        {{7,15},{102,102,102,100}},
        {{7,16},{102,102,102,100}},
        {{7,17},{0,0,1,100}},
        {{7,18},{102,102,102,100}},
        {{7,19},{0,0,1,100}},
        {{7,20},{102,102,102,100}},
        {{7,21},{0,0,1,100}},
        {{7,22},{102,102,102,100}},
        {{7,23},{0,0,1,100}},
        {{7,24},{102,102,102,100}},
        {{7,25},{102,102,102,100}},
        {{7,26},{102,102,102,100}},
        {{7,27},{0,0,1,100}},
        
        };

        public static final boolean enableCANcoder = true;
    }


}
