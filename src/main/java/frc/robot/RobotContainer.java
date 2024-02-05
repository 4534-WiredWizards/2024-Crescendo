package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.MoveArm;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick operatorJoystick = new Joystick(1);
        
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Limelight limelight = new Limelight(swerveSubsystem);
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climb climb = new Climb();
    private final Arm Arm = new Arm();




    public final static TrajectoryConfig autoTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    public RobotContainer() {
        // set pipeline
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
        
        shooter.setDefaultCommand(new RunShooter(shooter, intake,() -> operatorJoystick.getRawAxis(3), false));
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverThrottleAxis),
                () -> driverJoystick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
                ));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));




        // ----------------------- ARM COMMANDS ---------------------------------

        // Basic Operator Shooter Control
        // Right Trigger - Run Shooter (Speed changes based on how far the trigger is pressed)
  
        // Basic Operator Intake Control
        // Y - Run Intake
        new JoystickButton(operatorJoystick, InputDevices.btn_y).whileTrue(new RunIntake(intake, true, 7.5, true));

        // Basic Operator Arm Control
        // Bumper Left & Right (Left- Move arm twoards the intake, Right- Move arm away from intake)
        new JoystickButton(operatorJoystick ,InputDevices.btn_leftBumper).whileTrue(new MoveArm(Arm, .2));
        new JoystickButton(operatorJoystick, InputDevices.btn_rightBumper ).whileTrue(new MoveArm(Arm, -.2));

        //Lower arm position, run intake, move arm up if piece collected
        new POVButton(operatorJoystick, 180).onTrue(new SequentialCommandGroup(
                new PIDMoveArm(Arm, CommandConstants.intakeheight), //Lower arm
                new RunIntake(intake, true,.5, true), //Turn On Intake
                new PIDMoveArm(Arm, CommandConstants.traversalheight) //After run intake finishes (A piece is collected) move arm up
        ).until(() -> (operatorJoystick.getRawButtonPressed(5) || operatorJoystick.getRawButtonPressed(6))));

        //Move arm to amp height, spin up shooter, (Wont run intake tell button press)
        new POVButton(operatorJoystick, 90).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> limelight.resetLimelightPose()),
                new GeneralTrajectories().toTag(swerveSubsystem),
                new PIDMoveArm(Arm, CommandConstants.ampheight),
                new RunShooter(shooter, intake, () -> .9, true)
       ).until(() -> operatorJoystick.getRawButtonPressed(7)));


    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        // Note: -y value is to the left (field relative)
        // This sa,ple is an example of a figure 8 auto path
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(.7, -0.7),
                        new Translation2d(1.2, 0),
                        new Translation2d(.7, 0.7),
                        new Translation2d(0, 0),
                        new Translation2d(-.7, -0.7),
                        new Translation2d(-1.2, 0),
                        new Translation2d(-.7, 0.7)
                        ),
                new Pose2d(
               0, 0
                , Rotation2d.fromDegrees(0)),
                trajectoryConfig);      


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
