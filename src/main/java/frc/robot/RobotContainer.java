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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
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
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
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
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverThrottleAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
                ));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));


        //Sequential Command(s)

        //Lower arm and run intake
        new POVButton(operatorJoystick, 180).onTrue(new SequentialCommandGroup(
                new PIDMoveArm(Arm, 0.0),// test and change setpoint, remove this when done
                new RunIntake(intake, true,() ->.5, true),
                new PIDMoveArm(Arm, 0.0)// test and change setpoint, remove this when done
        ).until(() -> (operatorJoystick.getRawButtonPressed(5) || operatorJoystick.getRawButtonPressed(6))));


        //Shoot at amp(Center on april tag, move arm, shoot)
        new POVButton(operatorJoystick, 90).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> limelight.resetLimelightPose()),
                new GeneralTrajectories().toTag(swerveSubsystem),
                new PIDMoveArm(Arm, 0.0),// test and change setpoint, remove this when done
                new RunShooter(shooter, intake, .9, true)
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
