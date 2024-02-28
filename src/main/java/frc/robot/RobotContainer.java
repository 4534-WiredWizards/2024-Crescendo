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
import frc.robot.commands.PointToSpeaker;
import frc.robot.commands.PointToSpeaker2;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.runClimb;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    public final Joystick operatorJoystick = new Joystick(1);
        
    public final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    public final Climb climb = new Climb();
    public final Arm arm = new Arm();
    public final ArmProfiledPID ArmProfiledPID = new ArmProfiledPID(arm);
    public final SwerveSubsystem swerve = new SwerveSubsystem();
    private final Limelight limelight = new Limelight(swerve);
    public final AutoChooser autoChooser = new AutoChooser(swerve, shooter, arm, ArmProfiledPID, limelight, intake);




    public final static TrajectoryConfig autoTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    public RobotContainer() {
        // set pipeline
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        
        shooter.setDefaultCommand(new RunShooter(shooter, intake,() -> operatorJoystick.getRawAxis(3), false,false));
        
        swerve.setDefaultCommand(new SwerveJoystickCmd(
            swerve,
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
        new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerve.zeroHeading()));




        // ----------------------- ARM COMMANDS ---------------------------------

        // Basic Operator Shooter Control
        // Right Trigger - Run Shooter (Speed changes based on how far the trigger is pressed)
  
        // Basic Operator Intake Control
        // Y - Run Intake
        new JoystickButton(operatorJoystick, InputDevices.btn_y).whileTrue(new RunIntake(intake, true, .7, true));
        new JoystickButton(operatorJoystick, InputDevices.btn_a).whileTrue(new RunIntake(intake, true, -.7, true));
        // new JoystickButton(operatorJoystick, InputDevices.btn_x).onTrue(new PointToSpeaker(swerve, limelight));
        new JoystickButton(operatorJoystick, InputDevices.btn_x).whileTrue(new runClimb(0, climb));
        new JoystickButton(operatorJoystick, InputDevices.btn_b).whileTrue(new runClimb(510, climb));


        // new JoystickButton(operatorJoystick, InputDevices.btn_b).onTrue(new PointToSpeaker2(limelight, swerve));

        // new JoystickButton(operatorJoystick, InputDevices.btn_x).whileTrue(new RunShooter(shooter,.7, true));

        // Basic Operator Arm Control
        // Bumper Left & Right (Left- Move arm twoards the intake, Right- Move arm away from intake)
        new JoystickButton(operatorJoystick ,InputDevices.btn_leftBumper).whileTrue(new MoveArm(arm, .2));
        new JoystickButton(operatorJoystick, InputDevices.btn_rightBumper).whileTrue(new MoveArm(arm, -.20));

        //Lower arm position, run intake, move arm up if piece collected
        // new JoystickButton(operatorJoystick, InputDevices.btn_x).onTrue(new PIDMoveArm(arm, ArmProfiledPID, Units.degreesToRadians(30.0)));

        //new POVButton(operatorJoystick, 180).onTrue(new PIDMoveArm(arm, ArmProfiledPID, Units.degreesToRadians(-3.0)).until(() -> operatorJoystick.getRawButtonPressed(7)));
        new POVButton(operatorJoystick, 90).onTrue(new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(20.0)).until(() -> operatorJoystick.getRawButtonPressed(7)));
        new POVButton(operatorJoystick, 0).onTrue(new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(100.0)).until(() -> operatorJoystick.getRawButtonPressed(7)));
        new POVButton(operatorJoystick, 270).onTrue(new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(16)).until(() -> operatorJoystick.getRawButtonPressed(7)));


        //Move arm down to intake, then up
        new POVButton(operatorJoystick, 180).onTrue(new SequentialCommandGroup(
                new PIDMoveArm(arm, ArmProfiledPID,  Units.degreesToRadians(CommandConstants.Arm.intakeheight)), 
                new RunIntake(intake, true,.7, true), 
                new PIDMoveArm(arm, ArmProfiledPID,  Units.degreesToRadians(CommandConstants.Arm.traversalheight))
                // new InstantCommand(()->System.out.println("Step 5"))
        ));
        
        // .until(() -> (operatorJoystick.getRawButtonPressed(5) || operatorJoystick.getRawButtonPressed(6))));



        //Move arm to amp height, spin up shooter, (Wont run intake tell button press)
    //     new POVButton(operatorJoystick, 90).onTrue(new SequentialCommandGroup(
    //             new InstantCommand(() -> limelight.resetLimelightTargetPose()),
    //             new GeneralTrajectories().toTag(swerve),
    //             new PIDMoveArm(arm, ArmProfiledPID, CommandConstants.Arm.ampheight),
    //             new RunShooter(shooter, intake, () -> .9, false, true)
    //    ).until(() -> operatorJoystick.getRawButtonPressed(7)));


    }

    public Command getAutonomousCommand() {
        return autoChooser.getAuto();
    }
}
