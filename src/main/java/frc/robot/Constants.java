package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Lights.RGBColors;

public final class Constants {

  public static final class ModuleConstants {

    //The diameter of the wheel on your swerve drive
    public static final double kWheelDiameterMeters = Units.inchesToMeters(
      3.94
    );
    //The gear ratio for an SDS MK4 module with speed ratio of L2
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    //Ratio for an SDS MK4 turning motor
    public static final double kTurningMotorGearRatio = 1 / 12.80;
    public static final double kDriveEncoderRot2Meter =
      kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad =
      kTurningMotorGearRatio * 2 * Math.PI;
    public static final double upDOG = 36.0 / 36;
    public static final double kDriveEncoderRPM2MeterPerSec =
      kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec =
      kTurningEncoderRot2Rad / 60;
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
    //change later
    public static final int btn_xboxbutton = 10;

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
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),/*FL*/
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),/*FR*/
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),/*BL*/
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );/*BR*/

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
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(
      241.3
    );
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(
      85.34
    );
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(
      125.68
    );
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(
      196.4
    );

    // in m/s, based on MK4 L2 speed of 15.1 ft/s
    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(
      15.1
    );
    // Robot turning speed
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =
      5 * Math.PI;

    // If you want to slow down the robot during TeleOp, adjust these values
    public static final double kTeleDriveMaxSpeedMetersPerSecond =
      kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
      kPhysicalMaxAngularSpeedRadiansPerSecond / 4; // Slowed down for testing
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond =
      3;
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
      public static final double windSpeed = -.8;
      public static final double unwindSpeed = -windSpeed;
    }

    public static final class Intake {

      public static final double intakeSpeed = .5;
      public static final double outtakeSpeed = -.7;
      public static final double shootingSpeed = 1.0;
      public static final double autoIntakeSpeed = .4;
    }

    //Arm Commands
    public static final class Arm {

      public static final double AbsEncoderOffset = .85466;
      public static final double amp = 97.0;
      public static final double intake = .9;
      public static final double traversal = 65;
      public static final double closeSpeaker = 20;
      public static final double farSpeaker = 36.2; //Test and tune noteShot
      public static final double noteShot = 36.2;
      // Arm Feedforward Constants
      public static final double kP = 1;
      // Values Obtained from Characterization via reca.lc/arm

      // 10lbs Config
      // public static final double kSVolts = 1;
      // public static final double kGVolts = .45;
      // public static final double kVVoltSecondPerRad = 2.49;
      // public static final double kAVoltSecondSquaredPerRad = 0.03;

      // 20lbs Config
      // public static final double kSVolts = 1;
      // public static final double kGVolts = 0.90;
      // public static final double kVVoltSecondPerRad = 2.49;
      // public static final double kAVoltSecondSquaredPerRad = 0.03;

      // 23lbs, 130.5 gear ratio Config
      public static final double kSVolts = 1;
      public static final double kGVolts = 1.02;
      public static final double kVVoltSecondPerRad = 2.54;
      public static final double kAVoltSecondSquaredPerRad = 0.07;

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
    public static final double kMaxSpeedMetersPerSecond =
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond =
      DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;
    public static final double kMaxRotationalVelocityMetersPerSecond = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
      Math.PI / 2;
    // The P value of the PID controller used in auto for the X and Y directions
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    // The P value of the PID controller used in auto for the theta (rotation) direction
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularAccelerationRadiansPerSecondSquared
    );

    // Holonomic path following config
    public static HolonomicPathFollowerConfig AutoHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(5.5, 0, 0.0), // Translation PID constants
      new PIDConstants(2, 1, 0.0), // Rotation PID constants
      4.60, // Max module speed, in m/s
      0.3951, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig() // Default path replanning config. See the API for the options here
    );
  }

  public static final class TrajectoryConstants {

    // Constant position in meters for all game pieces
    // Note position closest to each driver station wall.
    // ----------- Blue Alliance -----------
    // Object Structure: X (In Meters), Y (In Meters), Angle (In Degrees)
    public static final Double centerOfSubwoofer = 1.4478;
    public static final Double angleToSubwoofer = 28.2685;

    public static final class blue {

      public static final double[] stageNote = { -5.540121, 0, 0 };
      public static final double[] speakerShoot = {
        -7.038721,
        centerOfSubwoofer,
        0,
      };
      public static final double[] speakerNote = {
        -5.540121,
        centerOfSubwoofer,
        0,
      };
      public static final double[] ampNote = { -5.540121, 2.8956, 28.2685 };
    }

    // ----------- Red Alliance -----------
    // Object Structure: X (In Meters), Y (In Meters), Angle (In Degrees)
    // Add 180 to the angle for the red alliance as robot intake needs to face center of field
    public static final class red {

      public static final double[] stageNote = { 5.616321, 0, 180 };
      public static final double[] speakerShoot = {
        7.114921,
        centerOfSubwoofer,
        180,
      };
      public static final double[] speakerNote = {
        5.616321,
        centerOfSubwoofer,
        180,
      };
      public static final double[] ampNote = { 5.616321, 2.8956, 180 };
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
    public static final int kDriverFieldOrientedButtonIdx = 4;
    // Button used to enable slow turning
    public static final int kDriverSlowTurnButtonIdx = 1;
    // Button used to reset the gyro to 0
    public static final int kDriverResetGyroButtonIdx = 2;
  }

  public static class LightsConstants {

    public static final int candlePort = 26;
  }

  public static class LightDesign {

    public static final int[][][] WIRED_WIZARDS = {
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
      { { 0, 7 }, RGBColors.ww.DARKGRAY },
      { { 0, 8 }, RGBColors.ww.DARKGRAY },
      { { 1, 6 }, RGBColors.ww.DARKGRAY },
      { { 1, 7 }, RGBColors.ww.DARKGRAY },
      { { 2, 5 }, RGBColors.ww.DARKGRAY },
      { { 2, 6 }, RGBColors.ww.DARKGRAY },
      { { 2, 7 }, RGBColors.ww.DARKGRAY },
      { { 2, 8 }, RGBColors.ww.DARKGRAY },
      { { 3, 7 }, RGBColors.ww.RED },
      { { 4, 6 }, RGBColors.ww.LIGHTGRAY },
      { { 5, 5 }, RGBColors.ww.LIGHTGRAY },
      { { 5, 6 }, RGBColors.ww.LIGHTGRAY },
      { { 5, 7 }, RGBColors.ww.LIGHTGRAY },
      { { 6, 6 }, RGBColors.ww.LIGHTGRAY },
      { { 6, 7 }, RGBColors.ww.LIGHTGRAY },
      { { 7, 6 }, RGBColors.ww.LIGHTGRAY },
      // 4534 Text
      { { 1, 12 }, RGBColors.ww.RED },
      { { 1, 14 }, RGBColors.ww.RED },
      { { 1, 16 }, RGBColors.ww.RED },
      { { 1, 17 }, RGBColors.ww.RED },
      { { 1, 18 }, RGBColors.ww.RED },
      { { 1, 20 }, RGBColors.ww.RED },
      { { 1, 21 }, RGBColors.ww.RED },
      { { 1, 22 }, RGBColors.ww.RED },
      { { 1, 24 }, RGBColors.ww.RED },
      { { 1, 26 }, RGBColors.ww.RED },
      { { 2, 12 }, RGBColors.ww.RED },
      { { 2, 14 }, RGBColors.ww.RED },
      { { 2, 16 }, RGBColors.ww.RED },
      { { 2, 22 }, RGBColors.ww.RED },
      { { 2, 24 }, RGBColors.ww.RED },
      { { 2, 26 }, RGBColors.ww.RED },
      { { 3, 12 }, RGBColors.ww.RED },
      { { 3, 14 }, RGBColors.ww.RED },
      { { 3, 16 }, RGBColors.ww.RED },
      { { 3, 22 }, RGBColors.ww.RED },
      { { 3, 24 }, RGBColors.ww.RED },
      { { 3, 26 }, RGBColors.ww.RED },
      { { 4, 12 }, RGBColors.ww.RED },
      { { 4, 13 }, RGBColors.ww.RED },
      { { 4, 14 }, RGBColors.ww.RED },
      { { 4, 16 }, RGBColors.ww.RED },
      { { 4, 17 }, RGBColors.ww.RED },
      { { 4, 18 }, RGBColors.ww.RED },
      { { 4, 20 }, RGBColors.ww.RED },
      { { 4, 21 }, RGBColors.ww.RED },
      { { 4, 22 }, RGBColors.ww.RED },
      { { 4, 24 }, RGBColors.ww.RED },
      { { 4, 25 }, RGBColors.ww.RED },
      { { 4, 26 }, RGBColors.ww.RED },
      { { 5, 14 }, RGBColors.ww.RED },
      { { 5, 18 }, RGBColors.ww.RED },
      { { 5, 22 }, RGBColors.ww.RED },
      { { 5, 26 }, RGBColors.ww.RED },
      { { 6, 14 }, RGBColors.ww.RED },
      { { 6, 18 }, RGBColors.ww.RED },
      { { 6, 22 }, RGBColors.ww.RED },
      { { 6, 26 }, RGBColors.ww.RED },
      { { 7, 14 }, RGBColors.ww.RED },
      { { 7, 16 }, RGBColors.ww.RED },
      { { 7, 17 }, RGBColors.ww.RED },
      { { 7, 18 }, RGBColors.ww.RED },
      { { 7, 20 }, RGBColors.ww.RED },
      { { 7, 21 }, RGBColors.ww.RED },
      { { 7, 22 }, RGBColors.ww.RED },
      { { 7, 26 }, RGBColors.ww.RED },
    };
    public static final int[][][] nCino = {
      { { 1, 7 }, RGBColors.BLACK },
      { { 1, 8 }, RGBColors.nCino.RED },
      { { 1, 9 }, RGBColors.BLACK },
      { { 1, 18 }, RGBColors.BLACK },
      { { 1, 19 }, RGBColors.nCino.TEXT },
      { { 1, 20 }, RGBColors.BLACK },
      { { 2, 7 }, RGBColors.BLACK },
      { { 2, 8 }, RGBColors.nCino.RED },
      { { 2, 9 }, RGBColors.BLACK },
      { { 2, 10 }, RGBColors.BLACK },
      { { 2, 14 }, RGBColors.BLACK },
      { { 2, 15 }, RGBColors.nCino.TEXT },
      { { 2, 16 }, RGBColors.nCino.TEXT },
      { { 2, 17 }, RGBColors.nCino.TEXT },
      { { 2, 18 }, RGBColors.BLACK },
      { { 2, 20 }, RGBColors.BLACK },
      { { 2, 21 }, RGBColors.nCino.TEXT },
      { { 2, 22 }, RGBColors.nCino.TEXT },
      { { 2, 23 }, RGBColors.nCino.TEXT },
      { { 2, 24 }, RGBColors.BLACK },
      { { 2, 25 }, RGBColors.nCino.TEXT },
      { { 2, 26 }, RGBColors.nCino.TEXT },
      { { 2, 27 }, RGBColors.nCino.TEXT },
      { { 2, 28 }, RGBColors.BLACK },
      { { 3, 5 }, RGBColors.BLACK },
      { { 3, 6 }, RGBColors.nCino.GREEN },
      { { 3, 7 }, RGBColors.BLACK },
      { { 3, 8 }, RGBColors.nCino.RED },
      { { 3, 9 }, RGBColors.BLACK },
      { { 3, 10 }, RGBColors.BLACK },
      { { 3, 11 }, RGBColors.nCino.TEXT },
      { { 3, 12 }, RGBColors.nCino.TEXT },
      { { 3, 13 }, RGBColors.nCino.TEXT },
      { { 3, 14 }, RGBColors.BLACK },
      { { 3, 15 }, RGBColors.nCino.TEXT },
      { { 3, 16 }, RGBColors.BLACK },
      { { 3, 18 }, RGBColors.BLACK },
      { { 3, 19 }, RGBColors.nCino.TEXT },
      { { 3, 20 }, RGBColors.BLACK },
      { { 3, 21 }, RGBColors.nCino.TEXT },
      { { 3, 22 }, RGBColors.BLACK },
      { { 3, 23 }, RGBColors.nCino.TEXT },
      { { 3, 24 }, RGBColors.BLACK },
      { { 3, 25 }, RGBColors.nCino.TEXT },
      { { 3, 26 }, RGBColors.BLACK },
      { { 3, 27 }, RGBColors.nCino.TEXT },
      { { 3, 28 }, RGBColors.BLACK },
      { { 4, 5 }, RGBColors.BLACK },
      { { 4, 6 }, RGBColors.nCino.GREEN },
      { { 4, 7 }, RGBColors.nCino.YELLOW },
      { { 4, 8 }, RGBColors.nCino.RED },
      { { 4, 9 }, RGBColors.BLACK },
      { { 4, 10 }, RGBColors.BLACK },
      { { 4, 11 }, RGBColors.nCino.TEXT },
      { { 4, 12 }, RGBColors.BLACK },
      { { 4, 13 }, RGBColors.nCino.TEXT },
      { { 4, 14 }, RGBColors.BLACK },
      { { 4, 15 }, RGBColors.nCino.TEXT },
      { { 4, 16 }, RGBColors.BLACK },
      { { 4, 18 }, RGBColors.BLACK },
      { { 4, 19 }, RGBColors.nCino.TEXT },
      { { 4, 20 }, RGBColors.BLACK },
      { { 4, 21 }, RGBColors.nCino.TEXT },
      { { 4, 22 }, RGBColors.BLACK },
      { { 4, 23 }, RGBColors.nCino.TEXT },
      { { 4, 24 }, RGBColors.BLACK },
      { { 4, 25 }, RGBColors.nCino.TEXT },
      { { 4, 26 }, RGBColors.BLACK },
      { { 4, 27 }, RGBColors.nCino.TEXT },
      { { 4, 28 }, RGBColors.BLACK },
      { { 5, 4 }, RGBColors.BLACK },
      { { 5, 5 }, RGBColors.nCino.BLUE },
      { { 5, 6 }, RGBColors.nCino.GREEN },
      { { 5, 7 }, RGBColors.nCino.YELLOW },
      { { 5, 8 }, RGBColors.nCino.RED },
      { { 5, 9 }, RGBColors.BLACK },
      { { 5, 10 }, RGBColors.BLACK },
      { { 5, 11 }, RGBColors.nCino.TEXT },
      { { 5, 12 }, RGBColors.BLACK },
      { { 5, 13 }, RGBColors.nCino.TEXT },
      { { 5, 14 }, RGBColors.BLACK },
      { { 5, 15 }, RGBColors.nCino.TEXT },
      { { 5, 16 }, RGBColors.BLACK },
      { { 5, 18 }, RGBColors.BLACK },
      { { 5, 19 }, RGBColors.nCino.TEXT },
      { { 5, 20 }, RGBColors.BLACK },
      { { 5, 21 }, RGBColors.nCino.TEXT },
      { { 5, 22 }, RGBColors.BLACK },
      { { 5, 23 }, RGBColors.nCino.TEXT },
      { { 5, 24 }, RGBColors.BLACK },
      { { 5, 25 }, RGBColors.nCino.TEXT },
      { { 5, 26 }, RGBColors.BLACK },
      { { 5, 27 }, RGBColors.nCino.TEXT },
      { { 5, 28 }, RGBColors.BLACK },
      { { 6, 4 }, RGBColors.BLACK },
      { { 6, 5 }, RGBColors.nCino.BLUE },
      { { 6, 6 }, RGBColors.nCino.GREEN },
      { { 6, 7 }, RGBColors.nCino.YELLOW },
      { { 6, 8 }, RGBColors.nCino.RED },
      { { 6, 9 }, RGBColors.BLACK },
      { { 6, 10 }, RGBColors.BLACK },
      { { 6, 11 }, RGBColors.nCino.TEXT },
      { { 6, 12 }, RGBColors.BLACK },
      { { 6, 13 }, RGBColors.nCino.TEXT },
      { { 6, 14 }, RGBColors.BLACK },
      { { 6, 15 }, RGBColors.nCino.TEXT },
      { { 6, 16 }, RGBColors.BLACK },
      { { 6, 18 }, RGBColors.BLACK },
      { { 6, 19 }, RGBColors.nCino.TEXT },
      { { 6, 20 }, RGBColors.BLACK },
      { { 6, 21 }, RGBColors.nCino.TEXT },
      { { 6, 22 }, RGBColors.BLACK },
      { { 6, 23 }, RGBColors.nCino.TEXT },
      { { 6, 24 }, RGBColors.BLACK },
      { { 6, 25 }, RGBColors.nCino.TEXT },
      { { 6, 26 }, RGBColors.BLACK },
      { { 6, 27 }, RGBColors.nCino.TEXT },
      { { 6, 28 }, RGBColors.BLACK },
      { { 7, 4 }, RGBColors.BLACK },
      { { 7, 5 }, RGBColors.nCino.BLUE },
      { { 7, 6 }, RGBColors.nCino.GREEN },
      { { 7, 7 }, RGBColors.nCino.YELLOW },
      { { 7, 8 }, RGBColors.nCino.RED },
      { { 7, 9 }, RGBColors.BLACK },
      { { 7, 10 }, RGBColors.BLACK },
      { { 7, 11 }, RGBColors.nCino.TEXT },
      { { 7, 12 }, RGBColors.BLACK },
      { { 7, 13 }, RGBColors.nCino.TEXT },
      { { 7, 14 }, RGBColors.BLACK },
      { { 7, 15 }, RGBColors.nCino.TEXT },
      { { 7, 16 }, RGBColors.nCino.TEXT },
      { { 7, 17 }, RGBColors.nCino.TEXT },
      { { 7, 18 }, RGBColors.BLACK },
      { { 7, 19 }, RGBColors.nCino.TEXT },
      { { 7, 20 }, RGBColors.BLACK },
      { { 7, 21 }, RGBColors.nCino.TEXT },
      { { 7, 22 }, RGBColors.BLACK },
      { { 7, 23 }, RGBColors.nCino.TEXT },
      { { 7, 24 }, RGBColors.BLACK },
      { { 7, 25 }, RGBColors.nCino.TEXT },
      { { 7, 26 }, RGBColors.nCino.TEXT },
      { { 7, 27 }, RGBColors.nCino.TEXT },
      { { 7, 28 }, RGBColors.BLACK },
    };

    public static final int[][][] Corning = {
      { { 1, 1 }, RGBColors.BLACK },
      { { 1, 2 }, RGBColors.Corning.TEXT },
      { { 1, 3 }, RGBColors.Corning.TEXT },
      { { 1, 4 }, RGBColors.Corning.TEXT },
      { { 1, 5 }, RGBColors.BLACK },
      { { 1, 6 }, RGBColors.Corning.TEXT },
      { { 1, 7 }, RGBColors.Corning.TEXT },
      { { 1, 8 }, RGBColors.Corning.TEXT },
      { { 1, 9 }, RGBColors.BLACK },
      { { 1, 10 }, RGBColors.Corning.TEXT },
      { { 1, 11 }, RGBColors.Corning.TEXT },
      { { 1, 12 }, RGBColors.Corning.TEXT },
      { { 1, 13 }, RGBColors.BLACK },
      { { 1, 14 }, RGBColors.Corning.TEXT },
      { { 1, 15 }, RGBColors.BLACK },
      { { 1, 16 }, RGBColors.BLACK },
      { { 1, 17 }, RGBColors.Corning.TEXT },
      { { 1, 18 }, RGBColors.BLACK },
      { { 1, 19 }, RGBColors.Corning.TEXT },
      { { 1, 20 }, RGBColors.BLACK },
      { { 1, 21 }, RGBColors.Corning.TEXT },
      { { 1, 22 }, RGBColors.BLACK },
      { { 1, 23 }, RGBColors.BLACK },
      { { 1, 24 }, RGBColors.Corning.TEXT },
      { { 1, 25 }, RGBColors.BLACK },
      { { 1, 26 }, RGBColors.Corning.TEXT },
      { { 1, 27 }, RGBColors.Corning.TEXT },
      { { 1, 28 }, RGBColors.Corning.TEXT },
      { { 1, 29 }, RGBColors.Corning.TEXT },
      { { 1, 30 }, RGBColors.BLACK },
      { { 2, 1 }, RGBColors.BLACK },
      { { 2, 2 }, RGBColors.Corning.TEXT },
      { { 2, 3 }, RGBColors.BLACK },
      { { 2, 5 }, RGBColors.BLACK },
      { { 2, 6 }, RGBColors.Corning.TEXT },
      { { 2, 7 }, RGBColors.BLACK },
      { { 2, 8 }, RGBColors.Corning.TEXT },
      { { 2, 9 }, RGBColors.BLACK },
      { { 2, 10 }, RGBColors.Corning.TEXT },
      { { 2, 11 }, RGBColors.BLACK },
      { { 2, 12 }, RGBColors.Corning.TEXT },
      { { 2, 13 }, RGBColors.BLACK },
      { { 2, 14 }, RGBColors.Corning.TEXT },
      { { 2, 15 }, RGBColors.Corning.TEXT },
      { { 2, 16 }, RGBColors.BLACK },
      { { 2, 17 }, RGBColors.Corning.TEXT },
      { { 2, 18 }, RGBColors.BLACK },
      { { 2, 19 }, RGBColors.BLACK },
      { { 2, 20 }, RGBColors.BLACK },
      { { 2, 21 }, RGBColors.Corning.TEXT },
      { { 2, 22 }, RGBColors.Corning.TEXT },
      { { 2, 23 }, RGBColors.BLACK },
      { { 2, 24 }, RGBColors.Corning.TEXT },
      { { 2, 25 }, RGBColors.BLACK },
      { { 2, 26 }, RGBColors.Corning.TEXT },
      { { 2, 27 }, RGBColors.BLACK },
      { { 2, 28 }, RGBColors.BLACK },
      { { 2, 29 }, RGBColors.BLACK },
      { { 2, 30 }, RGBColors.BLACK },
      { { 3, 1 }, RGBColors.BLACK },
      { { 3, 2 }, RGBColors.Corning.TEXT },
      { { 3, 3 }, RGBColors.BLACK },
      { { 3, 5 }, RGBColors.BLACK },
      { { 3, 6 }, RGBColors.Corning.TEXT },
      { { 3, 7 }, RGBColors.BLACK },
      { { 3, 8 }, RGBColors.Corning.TEXT },
      { { 3, 9 }, RGBColors.BLACK },
      { { 3, 10 }, RGBColors.Corning.TEXT },
      { { 3, 11 }, RGBColors.BLACK },
      { { 3, 12 }, RGBColors.Corning.TEXT },
      { { 3, 13 }, RGBColors.BLACK },
      { { 3, 14 }, RGBColors.Corning.TEXT },
      { { 3, 15 }, RGBColors.Corning.TEXT },
      { { 3, 16 }, RGBColors.Corning.TEXT },
      { { 3, 17 }, RGBColors.Corning.TEXT },
      { { 3, 18 }, RGBColors.BLACK },
      { { 3, 19 }, RGBColors.Corning.TEXT },
      { { 3, 20 }, RGBColors.BLACK },
      { { 3, 21 }, RGBColors.Corning.TEXT },
      { { 3, 22 }, RGBColors.Corning.TEXT },
      { { 3, 23 }, RGBColors.Corning.TEXT },
      { { 3, 24 }, RGBColors.Corning.TEXT },
      { { 3, 25 }, RGBColors.BLACK },
      { { 3, 26 }, RGBColors.Corning.TEXT },
      { { 3, 27 }, RGBColors.BLACK },
      { { 3, 28 }, RGBColors.BLACK },
      { { 3, 29 }, RGBColors.BLACK },
      { { 3, 30 }, RGBColors.BLACK },
      { { 4, 1 }, RGBColors.BLACK },
      { { 4, 2 }, RGBColors.Corning.TEXT },
      { { 4, 3 }, RGBColors.BLACK },
      { { 4, 5 }, RGBColors.BLACK },
      { { 4, 6 }, RGBColors.Corning.TEXT },
      { { 4, 7 }, RGBColors.BLACK },
      { { 4, 8 }, RGBColors.Corning.TEXT },
      { { 4, 9 }, RGBColors.BLACK },
      { { 4, 10 }, RGBColors.Corning.TEXT },
      { { 4, 11 }, RGBColors.Corning.TEXT },
      { { 4, 12 }, RGBColors.BLACK },
      { { 4, 13 }, RGBColors.BLACK },
      { { 4, 14 }, RGBColors.Corning.TEXT },
      { { 4, 15 }, RGBColors.BLACK },
      { { 4, 16 }, RGBColors.Corning.TEXT },
      { { 4, 17 }, RGBColors.Corning.TEXT },
      { { 4, 18 }, RGBColors.BLACK },
      { { 4, 19 }, RGBColors.Corning.TEXT },
      { { 4, 20 }, RGBColors.BLACK },
      { { 4, 21 }, RGBColors.Corning.TEXT },
      { { 4, 22 }, RGBColors.BLACK },
      { { 4, 23 }, RGBColors.Corning.TEXT },
      { { 4, 24 }, RGBColors.Corning.TEXT },
      { { 4, 25 }, RGBColors.BLACK },
      { { 4, 26 }, RGBColors.Corning.TEXT },
      { { 4, 27 }, RGBColors.BLACK },
      { { 4, 28 }, RGBColors.Corning.TEXT },
      { { 4, 29 }, RGBColors.Corning.TEXT },
      { { 4, 30 }, RGBColors.BLACK },
      { { 5, 1 }, RGBColors.BLACK },
      { { 5, 2 }, RGBColors.Corning.TEXT },
      { { 5, 3 }, RGBColors.BLACK },
      { { 5, 5 }, RGBColors.BLACK },
      { { 5, 6 }, RGBColors.Corning.TEXT },
      { { 5, 7 }, RGBColors.BLACK },
      { { 5, 8 }, RGBColors.Corning.TEXT },
      { { 5, 9 }, RGBColors.BLACK },
      { { 5, 10 }, RGBColors.Corning.TEXT },
      { { 5, 11 }, RGBColors.BLACK },
      { { 5, 12 }, RGBColors.Corning.TEXT },
      { { 5, 13 }, RGBColors.BLACK },
      { { 5, 14 }, RGBColors.Corning.TEXT },
      { { 5, 15 }, RGBColors.BLACK },
      { { 5, 16 }, RGBColors.Corning.TEXT },
      { { 5, 17 }, RGBColors.Corning.TEXT },
      { { 5, 18 }, RGBColors.BLACK },
      { { 5, 19 }, RGBColors.Corning.TEXT },
      { { 5, 20 }, RGBColors.BLACK },
      { { 5, 21 }, RGBColors.Corning.TEXT },
      { { 5, 22 }, RGBColors.BLACK },
      { { 5, 23 }, RGBColors.Corning.TEXT },
      { { 5, 24 }, RGBColors.Corning.TEXT },
      { { 5, 25 }, RGBColors.BLACK },
      { { 5, 26 }, RGBColors.Corning.TEXT },
      { { 5, 27 }, RGBColors.BLACK },
      { { 5, 28 }, RGBColors.BLACK },
      { { 5, 29 }, RGBColors.Corning.TEXT },
      { { 5, 30 }, RGBColors.BLACK },
      { { 6, 1 }, RGBColors.BLACK },
      { { 6, 2 }, RGBColors.Corning.TEXT },
      { { 6, 3 }, RGBColors.BLACK },
      { { 6, 5 }, RGBColors.BLACK },
      { { 6, 6 }, RGBColors.Corning.TEXT },
      { { 6, 7 }, RGBColors.BLACK },
      { { 6, 8 }, RGBColors.Corning.TEXT },
      { { 6, 9 }, RGBColors.BLACK },
      { { 6, 10 }, RGBColors.Corning.TEXT },
      { { 6, 11 }, RGBColors.BLACK },
      { { 6, 12 }, RGBColors.Corning.TEXT },
      { { 6, 13 }, RGBColors.BLACK },
      { { 6, 14 }, RGBColors.Corning.TEXT },
      { { 6, 15 }, RGBColors.BLACK },
      { { 6, 16 }, RGBColors.BLACK },
      { { 6, 17 }, RGBColors.Corning.TEXT },
      { { 6, 18 }, RGBColors.BLACK },
      { { 6, 19 }, RGBColors.Corning.TEXT },
      { { 6, 20 }, RGBColors.BLACK },
      { { 6, 21 }, RGBColors.Corning.TEXT },
      { { 6, 22 }, RGBColors.BLACK },
      { { 6, 23 }, RGBColors.BLACK },
      { { 6, 24 }, RGBColors.Corning.TEXT },
      { { 6, 25 }, RGBColors.BLACK },
      { { 6, 26 }, RGBColors.Corning.TEXT },
      { { 6, 27 }, RGBColors.BLACK },
      { { 6, 28 }, RGBColors.BLACK },
      { { 6, 29 }, RGBColors.Corning.TEXT },
      { { 6, 30 }, RGBColors.BLACK },
      { { 7, 1 }, RGBColors.BLACK },
      { { 7, 2 }, RGBColors.Corning.TEXT },
      { { 7, 3 }, RGBColors.Corning.TEXT },
      { { 7, 4 }, RGBColors.Corning.TEXT },
      { { 7, 5 }, RGBColors.BLACK },
      { { 7, 6 }, RGBColors.Corning.TEXT },
      { { 7, 7 }, RGBColors.Corning.TEXT },
      { { 7, 8 }, RGBColors.Corning.TEXT },
      { { 7, 9 }, RGBColors.BLACK },
      { { 7, 10 }, RGBColors.Corning.TEXT },
      { { 7, 11 }, RGBColors.BLACK },
      { { 7, 12 }, RGBColors.Corning.TEXT },
      { { 7, 13 }, RGBColors.BLACK },
      { { 7, 14 }, RGBColors.Corning.TEXT },
      { { 7, 15 }, RGBColors.BLACK },
      { { 7, 16 }, RGBColors.BLACK },
      { { 7, 17 }, RGBColors.Corning.TEXT },
      { { 7, 18 }, RGBColors.BLACK },
      { { 7, 19 }, RGBColors.Corning.TEXT },
      { { 7, 20 }, RGBColors.BLACK },
      { { 7, 21 }, RGBColors.Corning.TEXT },
      { { 7, 22 }, RGBColors.BLACK },
      { { 7, 23 }, RGBColors.BLACK },
      { { 7, 24 }, RGBColors.Corning.TEXT },
      { { 7, 25 }, RGBColors.BLACK },
      { { 7, 26 }, RGBColors.Corning.TEXT },
      { { 7, 27 }, RGBColors.Corning.TEXT },
      { { 7, 28 }, RGBColors.Corning.TEXT },
      { { 7, 29 }, RGBColors.Corning.TEXT },
      { { 7, 30 }, RGBColors.BLACK },
    };

    public static final int[][][] CFCC = {
      { { 1, 6 }, RGBColors.BLACK },
      { { 1, 7 }, RGBColors.CFCC.TEXT },
      { { 1, 8 }, RGBColors.CFCC.TEXT },
      { { 1, 9 }, RGBColors.BLACK },
      { { 1, 11 }, RGBColors.BLACK },
      { { 1, 12 }, RGBColors.CFCC.TEXT },
      { { 1, 13 }, RGBColors.CFCC.TEXT },
      { { 1, 14 }, RGBColors.CFCC.TEXT },
      { { 1, 15 }, RGBColors.CFCC.TEXT },
      { { 1, 16 }, RGBColors.BLACK },
      { { 1, 17 }, RGBColors.BLACK },
      { { 1, 18 }, RGBColors.CFCC.TEXT },
      { { 1, 19 }, RGBColors.CFCC.TEXT },
      { { 1, 20 }, RGBColors.BLACK },
      { { 1, 22 }, RGBColors.BLACK },
      { { 1, 23 }, RGBColors.CFCC.TEXT },
      { { 1, 24 }, RGBColors.CFCC.TEXT },
      { { 1, 25 }, RGBColors.BLACK },
      { { 2, 5 }, RGBColors.BLACK },
      { { 2, 6 }, RGBColors.CFCC.TEXT },
      { { 2, 7 }, RGBColors.BLACK },
      { { 2, 8 }, RGBColors.BLACK },
      { { 2, 9 }, RGBColors.CFCC.TEXT },
      { { 2, 10 }, RGBColors.BLACK },
      { { 2, 11 }, RGBColors.BLACK },
      { { 2, 12 }, RGBColors.CFCC.TEXT },
      { { 2, 13 }, RGBColors.BLACK },
      { { 2, 16 }, RGBColors.BLACK },
      { { 2, 17 }, RGBColors.CFCC.TEXT },
      { { 2, 18 }, RGBColors.BLACK },
      { { 2, 19 }, RGBColors.BLACK },
      { { 2, 20 }, RGBColors.CFCC.TEXT },
      { { 2, 21 }, RGBColors.BLACK },
      { { 2, 22 }, RGBColors.CFCC.TEXT },
      { { 2, 23 }, RGBColors.BLACK },
      { { 2, 24 }, RGBColors.BLACK },
      { { 2, 25 }, RGBColors.CFCC.TEXT },
      { { 2, 26 }, RGBColors.BLACK },
      { { 3, 5 }, RGBColors.BLACK },
      { { 3, 6 }, RGBColors.CFCC.TEXT },
      { { 3, 7 }, RGBColors.BLACK },
      { { 3, 10 }, RGBColors.BLACK },
      { { 3, 11 }, RGBColors.CFCC.TEXT },
      { { 3, 12 }, RGBColors.CFCC.TEXT },
      { { 3, 13 }, RGBColors.BLACK },
      { { 3, 16 }, RGBColors.BLACK },
      { { 3, 17 }, RGBColors.CFCC.TEXT },
      { { 3, 18 }, RGBColors.BLACK },
      { { 3, 21 }, RGBColors.BLACK },
      { { 3, 22 }, RGBColors.CFCC.TEXT },
      { { 3, 23 }, RGBColors.BLACK },
      { { 4, 5 }, RGBColors.BLACK },
      { { 4, 6 }, RGBColors.CFCC.TEXT },
      { { 4, 7 }, RGBColors.BLACK },
      { { 4, 11 }, RGBColors.BLACK },
      { { 4, 12 }, RGBColors.CFCC.TEXT },
      { { 4, 13 }, RGBColors.CFCC.TEXT },
      { { 4, 14 }, RGBColors.CFCC.TEXT },
      { { 4, 15 }, RGBColors.BLACK },
      { { 4, 16 }, RGBColors.BLACK },
      { { 4, 17 }, RGBColors.CFCC.TEXT },
      { { 4, 18 }, RGBColors.BLACK },
      { { 4, 21 }, RGBColors.BLACK },
      { { 4, 22 }, RGBColors.CFCC.TEXT },
      { { 4, 23 }, RGBColors.BLACK },
      { { 5, 5 }, RGBColors.BLACK },
      { { 5, 6 }, RGBColors.CFCC.TEXT },
      { { 5, 7 }, RGBColors.BLACK },
      { { 5, 11 }, RGBColors.BLACK },
      { { 5, 12 }, RGBColors.CFCC.TEXT },
      { { 5, 13 }, RGBColors.BLACK },
      { { 5, 16 }, RGBColors.BLACK },
      { { 5, 17 }, RGBColors.CFCC.TEXT },
      { { 5, 18 }, RGBColors.BLACK },
      { { 5, 21 }, RGBColors.BLACK },
      { { 5, 22 }, RGBColors.CFCC.TEXT },
      { { 5, 23 }, RGBColors.BLACK },
      { { 6, 5 }, RGBColors.BLACK },
      { { 6, 6 }, RGBColors.CFCC.TEXT },
      { { 6, 7 }, RGBColors.BLACK },
      { { 6, 8 }, RGBColors.BLACK },
      { { 6, 9 }, RGBColors.CFCC.TEXT },
      { { 6, 10 }, RGBColors.BLACK },
      { { 6, 11 }, RGBColors.BLACK },
      { { 6, 12 }, RGBColors.CFCC.TEXT },
      { { 6, 13 }, RGBColors.BLACK },
      { { 6, 16 }, RGBColors.BLACK },
      { { 6, 17 }, RGBColors.CFCC.TEXT },
      { { 6, 18 }, RGBColors.BLACK },
      { { 6, 19 }, RGBColors.BLACK },
      { { 6, 20 }, RGBColors.CFCC.TEXT },
      { { 6, 21 }, RGBColors.BLACK },
      { { 6, 22 }, RGBColors.CFCC.TEXT },
      { { 6, 23 }, RGBColors.BLACK },
      { { 6, 24 }, RGBColors.BLACK },
      { { 6, 25 }, RGBColors.CFCC.TEXT },
      { { 6, 26 }, RGBColors.BLACK },
      { { 7, 6 }, RGBColors.BLACK },
      { { 7, 7 }, RGBColors.CFCC.TEXT },
      { { 7, 8 }, RGBColors.CFCC.TEXT },
      { { 7, 9 }, RGBColors.BLACK },
      { { 7, 11 }, RGBColors.BLACK },
      { { 7, 12 }, RGBColors.CFCC.TEXT },
      { { 7, 13 }, RGBColors.BLACK },
      { { 7, 17 }, RGBColors.BLACK },
      { { 7, 18 }, RGBColors.CFCC.TEXT },
      { { 7, 19 }, RGBColors.CFCC.TEXT },
      { { 7, 20 }, RGBColors.BLACK },
      { { 7, 22 }, RGBColors.BLACK },
      { { 7, 23 }, RGBColors.CFCC.TEXT },
      { { 7, 24 }, RGBColors.CFCC.TEXT },
      { { 7, 25 }, RGBColors.BLACK },
    };
  }
}
