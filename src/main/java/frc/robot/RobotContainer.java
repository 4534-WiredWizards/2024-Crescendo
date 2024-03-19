package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.PointToSpeaker2;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.manualClimb;
import frc.robot.commands.runClimb;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class RobotContainer {

  private final Joystick driverJoystick = new Joystick(
    OIConstants.kDriverControllerPort
  );
  public final Joystick operatorJoystick = new Joystick(1);

  public final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  public final Climb climb = new Climb();
  public final Arm arm = new Arm();
  public final ArmProfiledPID ArmProfiledPID = new ArmProfiledPID(arm);
  public final SwerveSubsystem swerve = new SwerveSubsystem();
  public final Limelight limelight = new Limelight(swerve);
  public static final Lights leds = new Lights();
  public final AutoChooser autoChooser = new AutoChooser(
    swerve,
    shooter,
    intake,
    arm,
    ArmProfiledPID,
    limelight
  );

  public static final TrajectoryConfig autoTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared
  )
    .setKinematics(DriveConstants.kDriveKinematics);

  public RobotContainer() {
    // set pipeline
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(1);

    shooter.setDefaultCommand(
      new RunShooter(
        shooter,
        intake,
        () -> operatorJoystick.getRawAxis(3),
        false,
        false,
        operatorJoystick.getRawButton(InputDevices.btn_y)
      )
    );

    swerve.setDefaultCommand(
      new SwerveJoystickCmd(
        swerve,
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () ->
          !driverJoystick.getRawButton(
            OIConstants.kDriverFieldOrientedButtonIdx
          ),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverThrottleAxis),
        () -> driverJoystick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
      )
    );

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // ----------------------- DRIVER COMMANDS ---------------------------------
    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonIdx)
      .onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    // ----------------------- SHOOTER COMMANDS ---------------------------------
    // Menu Button - Run Shooter at set velocity
    new JoystickButton(operatorJoystick, InputDevices.btn_select)
      .whileTrue(
        new RunShooter(
          shooter,
          intake,
          () -> SmartDashboard.getNumber("Set Shooter Speed", 0),
          true,
          false,
          false
        )
      );

    // ----------------------- INTAKE & Shooter COMMANDS ---------------------------------
    // Basic Operator Intake Control
    // Y - Run Intake
    new JoystickButton(operatorJoystick, InputDevices.btn_y)
      .whileTrue(
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.shootingSpeed,
          true
        )
      );
    new JoystickButton(operatorJoystick, InputDevices.btn_a)
      .whileTrue(
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.outtakeSpeed,
          true
        )
      );
    // Testing Auto Speaker Scoring System
    // new JoystickButton(operatorJoystick, InputDevices.btn_b).onTrue(new PointToSpeaker2(limelight, swerve));
    // new JoystickButton(operatorJoystick, InputDevices.btn_b).onTrue(new ArmToShootingH(limelight, swerve));

    // ----------------------- CLIMB COMMANDS ---------------------------------
    new JoystickButton(operatorJoystick, InputDevices.btn_x)
      .whileTrue(new runClimb(0, climb));
    new JoystickButton(operatorJoystick, InputDevices.btn_b)
      .whileTrue(new runClimb(510, climb));
    // new JoystickButton(operatorJoystick, InputDevices.btn_x).whileTrue(new manualClimb(true, climb));
    // new JoystickButton(operatorJoystick, InputDevices.btn_x).whileTrue(new manualClimb(false, climb));

    // ----------------------- ARM COMMANDS ---------------------------------

    // Basic Operator Arm Control
    // Bumper Left & Right (Left- Move arm twoards the intake, Right- Move arm away from intake)
    new JoystickButton(operatorJoystick, InputDevices.btn_leftBumper)
      .whileTrue(new MoveArm(arm, -.25)); //Move arm down
    new JoystickButton(operatorJoystick, InputDevices.btn_rightBumper)
      .whileTrue(new MoveArm(arm, .50)); //Move arm

    // ----------------------- ARM PID COMMANDS ---------------------------------

    // Right D Pad - Move arm to traversal position
    new POVButton(operatorJoystick, 90)
      .onTrue(
        new PIDMoveArm(
          arm,
          ArmProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.traversal),
          false
        )
          .until(() -> operatorJoystick.getRawButtonPressed(7))
      );
    // Up D Pad - Move arm to amp position
    new POVButton(operatorJoystick, 0)
      .onTrue(
        new PIDMoveArm(
          arm,
          ArmProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.amp),
          false
        )
          .until(() -> operatorJoystick.getRawButtonPressed(7))
      );
    // Left D Pad - Move arm to close speaker position
    new POVButton(operatorJoystick, 270)
      .onTrue(
        new PIDMoveArm(
          arm,
          ArmProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
          false
        )
          .until(() -> operatorJoystick.getRawButtonPressed(7))
      );

    // Select Button - Move arm to close speaker position
    new JoystickButton(operatorJoystick, InputDevices.btn_select)
      .onTrue(
        new PIDMoveArm(
          arm,
          ArmProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
          false
        )
      );
    // Start Button - Move arm to long shot position
    new JoystickButton(operatorJoystick, InputDevices.btn_start)
      .onTrue(
        new PIDMoveArm(
          arm,
          ArmProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.farSpeaker),
          false
        )
      );

    // Down D Pad - Move arm to intake position
    new POVButton(operatorJoystick, 180)
      .onTrue(
        new SequentialCommandGroup(
          // Move arm down, run intake, move arm back up (once intake is full)
          new PIDMoveArm(
            arm,
            ArmProfiledPID,
            Units.degreesToRadians(CommandConstants.Arm.intake),
            true
          ),
          new RunIntake(
            intake,
            true,
            Constants.CommandConstants.Intake.intakeSpeed,
            true
          ),
          new PIDMoveArm(
            arm,
            ArmProfiledPID,
            Units.degreesToRadians(CommandConstants.Arm.traversal),
            false
          )
        )
      );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getAuto();
  }
  // // Code to get Alliance color
  // public static String getAllianceColor() {
  // Fetch the alliance color
  // return "Red";
  // Optional<Alliance> ally = DriverStation.getAlliance();
  // if (ally.isPresent()) {
  //     if (ally.get() == Alliance.Red) {
  //         return "Red";
  //     }
  //     if (ally.get() == Alliance.Blue) {
  //         return "Blue";
  //     }
  //     else {
  //         return "Error";
  //     }
  // }
  // else {
  //     // Throw Error
  //     return "Error";
  // }
  // }
}
