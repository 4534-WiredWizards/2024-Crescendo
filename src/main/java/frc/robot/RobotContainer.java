package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoRoutines.autoShoot;
import frc.robot.commands.CalculateArmAngle;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.PointToSpeaker2;
import frc.robot.commands.RampUpShooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveJoystickCmd;
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
import javax.print.attribute.standard.MediaSize.NA;

public class RobotContainer {

  private final Joystick driverJoystick = new Joystick(
    OIConstants.kDriverControllerPort
  );
  public final Joystick operatorJoystick = new Joystick(1);

  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();
  public final Arm arm = new Arm();
  public final ArmProfiledPID armProfiledPID = new ArmProfiledPID(arm);
  public final SwerveSubsystem swerve = new SwerveSubsystem();
  public final Limelight limelight = new Limelight(swerve);
  // public final Climb climb = new Climb();
  // public static final Lights leds = new Lights();
  public final AutoChooser autoChooser = new AutoChooser(
    swerve,
    shooter,
    intake,
    arm,
    armProfiledPID,
    limelight
  );
  private final SendableChooser<Command> autoChooserTwo;

  public static final TrajectoryConfig autoTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared
  )
    .setKinematics(DriveConstants.kDriveKinematics);

  public RobotContainer() {
    // Add Named Commands for path planner

    NamedCommands.registerCommand(
      "Intake",
      new frc.robot.commands.AutoRoutines.intake(arm, armProfiledPID, intake)
    );

    //Reset the botpose
    NamedCommands.registerCommand(
      "ResetBotPose",
      new InstantCommand(() -> limelight.botposeblue.resetLimelightBotPoseBlue()
      )
    );

    NamedCommands.registerCommand(
      "ShootOnSub",
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new PIDMoveArm(
            arm,
            armProfiledPID,
            Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
            true
          ),
          new RampUpShooter(shooter)
        ),
        new Shoot(shooter, intake)
      )
    );
    NamedCommands.registerCommand("StartShooter", new RampUpShooter(shooter));
    NamedCommands.registerCommand(
      "AutoShoot",
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter)
    );

    NamedCommands.registerCommand(
      "PID-Speaker",
      new PIDMoveArm(
        arm,
        armProfiledPID,
        Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
        false
      )
    );

    NamedCommands.registerCommand(
      "PID-Amp",
      new PIDMoveArm(
        arm,
        armProfiledPID,
        Units.degreesToRadians(CommandConstants.Arm.amp),
        true
      )
    );

    NamedCommands.registerCommand(
      "PID-Traversal",
      new PIDMoveArm(
        arm,
        armProfiledPID,
        Units.degreesToRadians(CommandConstants.Arm.traversal),
        true
      )
    );

    NamedCommands.registerCommand("RampUpShooter", new RampUpShooter(shooter));

    //shoot command
    NamedCommands.registerCommand("Shoot", new Shoot(shooter, intake));

    // set pipeline

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

    autoChooserTwo = AutoBuilder.buildAutoChooser();
    // Clear Existing entries
    SmartDashboard.putData("Path Planner #1", autoChooserTwo);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // ----------------------- DRIVER COMMANDS ---------------------------------
    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonIdx)
      .onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    // ----------------------- SHOOTER COMMANDS ---------------------------------
    // Select Button - AUTO SHOOT - Face the speaker, calculate arm angle, spin up shooter, run intake, take shot
    // Highly recommend using this button for shooting over PID close speaker shooting
    // Will take over robot rotation and arm angle
    // Can be cancelled by pressing the Start button
    new JoystickButton(operatorJoystick, InputDevices.btn_select)
      .onTrue(
        new SequentialCommandGroup(
          new InstantCommand(() -> System.out.println("Auto Shoot")),
          new ParallelDeadlineGroup(
            new ParallelCommandGroup(
              new PointToSpeaker2(limelight, swerve),
              new CalculateArmAngle(arm, armProfiledPID, limelight)
            ),
            new RampUpShooter(shooter)
          ),
          new RunShooter(shooter, intake, () -> 1.0, false, true, true)
        )
          .until(() ->
            operatorJoystick.getRawButtonPressed(InputDevices.btn_start) // Btn to cancel all autoshoot commands
          )
      );

    // ----------------------- INTAKE & Shooter COMMANDS ---------------------------------
    // Basic Operator Intake Control
    // Y - Run Intake in
    new JoystickButton(operatorJoystick, InputDevices.btn_y)
      .whileTrue(
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.shootingSpeed,
          true
        )
      );

    // A - Run Intake out
    new JoystickButton(operatorJoystick, InputDevices.btn_a)
      .whileTrue(
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.outtakeSpeed,
          true
        )
      );

    // ----------------------- CLIMB COMMANDS ---------------------------------
    // new JoystickButton(operatorJoystick, InputDevices.btn_x)
    //   .whileTrue(new runClimb(0, climb));
    // new JoystickButton(operatorJoystick, InputDevices.btn_b)
    //   .whileTrue(new runClimb(510, climb));
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
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.traversal), //Changed back from far speaker
          false
        )
          .until(() -> operatorJoystick.getRawButtonPressed(7))
      );

    // Up D Pad - Move arm to amp position
    new POVButton(operatorJoystick, 0)
      .onTrue(
        new PIDMoveArm(
          arm,
          armProfiledPID,
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
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
          false
        )
          .until(() -> operatorJoystick.getRawButtonPressed(7))
      );

    // Select Button - Move arm to close speaker position
    // new JoystickButton(operatorJoystick, InputDevices.btn_select)
    //   .onTrue(
    //     new PIDMoveArm(
    //       arm,
    //       ArmProfiledPID,
    //       Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
    //       false
    //     )
    //   );

    // Start Button - Move arm to long shot position
    // new JoystickButton(operatorJoystick, InputDevices.btn_start)
    //   .onTrue(
    //     new PIDMoveArm(
    //       arm,
    //       ArmProfiledPID,
    //       Units.degreesToRadians(CommandConstants.Arm.traversal),
    //       false
    //     )
    //   );

    // Down D Pad - Move arm to intake position
    new POVButton(operatorJoystick, 180)
      .onTrue(
        new SequentialCommandGroup(
          // Move arm down, run intake, move arm back up (once intake is full)
          new PIDMoveArm(
            arm,
            armProfiledPID,
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
            armProfiledPID,
            Units.degreesToRadians(CommandConstants.Arm.traversal),
            false
          )
        )
      );
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getAuto(); //Old AutoChooser

    // Reset Botpose basesd on Limelight botpose blue
    Double botXPose = limelight.botposeblue.getXDistance();
    Double botYPose = limelight.botposeblue.getYDistance();
    Double botRotation = limelight.botposeblue.getThetaDegreesField();
    System.out.println("Reseting Bot Pose In AUTO");
    limelight.resetLimelightBotPose(botXPose, botYPose, botRotation);

    return autoChooserTwo.getSelected(); //New AutoChooser with path planner
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
