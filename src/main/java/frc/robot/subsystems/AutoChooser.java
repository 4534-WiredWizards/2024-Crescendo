// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommandConstants;
import frc.robot.GeneralTrajectories;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{
    RedMiddleNote,
    BlueMiddleNote,
    DoNothing
  }
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter shooter;
  private final Arm arm;
  private final ArmProfiledPID ArmProfiledPID;
  private final Intake intake;
  private final Limelight limelight;
  private SendableChooser<AutoMode> autoChooser;
  private Command autoRoutine;
  /** Creates a new AutoChooser. */
  public AutoChooser(
    SwerveSubsystem SwerveSubsystem,
    Shooter Shooter,
    Arm Arm,
    ArmProfiledPID ArmProfiledPID,
    Limelight Limelight,
    Intake intake
  ) {
    this.swerveSubsystem = SwerveSubsystem;
    this.shooter = Shooter;
    this.arm = Arm;
    this.ArmProfiledPID = ArmProfiledPID;
    this.intake = intake;
    this.limelight = Limelight;
    autoChooser = new SendableChooser<AutoMode>();
    autoChooser.addOption("Red Middle Note", AutoMode.RedMiddleNote);
    autoChooser.addOption("Blue Middle Note", AutoMode.BlueMiddleNote);
    autoChooser.addOption("Do Nothing", AutoMode.DoNothing);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAuto(){
    AutoMode selectedAutoMode = (AutoMode) (autoChooser.getSelected());
    System.out.println("Running getAuto");
    switch (selectedAutoMode) {
      default:
      case RedMiddleNote:
        System.out.println("Starting Middle Red Note"); 
        limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
        System.out.println("After Odom Reset"); 
        autoRoutine = new SequentialCommandGroup(
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redMiddleNote, true)
        );
      break;
      case BlueMiddleNote:
        System.out.println("Starting Middle Blue Note"); 
        limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
        System.out.println("After Odom Reset"); 
        autoRoutine = new SequentialCommandGroup(
            // Trajectory to center and take a shot
            // new FollowTrajectory(swerveSubsystem, AutoTrajectories.middleBlueSpeaker, true),

            // --------- Shoots single preloaded note into speaker ---------
            new PIDMoveArm(arm, ArmProfiledPID,  Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)), 
            new RunShooter(shooter, intake, () -> 1.0, false,true, true), //Shoots the ring automatically
            new PIDMoveArm(arm, ArmProfiledPID,  Units.degreesToRadians(CommandConstants.Arm.intake)), //Move arm down



            // --------- Drive to the middle note ---------
            new ParallelCommandGroup(
              new RunIntake(intake, true,.7, true),
              new SequentialCommandGroup(
                new DoNothing().withTimeout(.2),
                new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueMiddleNote, true)
              )
            ),
            new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueMiddleNote, true),
              new PIDMoveArm(arm, ArmProfiledPID,  Units.degreesToRadians(CommandConstants.Arm.closeSpeaker))
            )
        );
      break;



      // case MultiSpeaker:
      // //Specific setpoints not finished, remove this line when tested and finished
      // autoRoutine = new SequentialCommandGroup(
        
      // );
      // break;

      // case AmpScore:
      //  //Specific setpoints not finished, remove this line when tested and finished
      // autoRoutine = new SequentialCommandGroup(
      //   // new InstantCommand(() -> limelight.resetLimelightPose()),
      //   new GeneralTrajectories().toTag(swerveSubsystem),
      //   new PIDMoveArm(arm, ArmProfiledPID, 0.0)
      //   // new RunShooter(shooter, intake, () -> 1.0, false,true)
      // );
      // break;

      // case Driveout:
      // autoRoutine = new GeneralTrajectories().Back(swerveSubsystem);
      // break;

      case DoNothing:
      autoRoutine = new DoNothing();
  }
  return autoRoutine;
}
}

