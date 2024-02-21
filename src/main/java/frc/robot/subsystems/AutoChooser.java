// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GeneralTrajectories;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{
    MultiSpeaker,
    AmpScore,
    Driveout,
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
      case MultiSpeaker:
      //Specific setpoints not finished, remove this line when tested and finished
      autoRoutine = new SequentialCommandGroup(
        // new InstantCommand(() -> limelight.resetLimelightPose()),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new PIDMoveArm(arm,ArmProfiledPID, 0.0),
        new RunShooter(shooter, intake, () -> 1.0, false,true),
        new ParallelCommandGroup(
          new GeneralTrajectories().toStraightBackNote(swerveSubsystem),
          // new PIDMoveArm(arm, 0.0),
          new RunIntake(intake, true, 1.0, true)
        ),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new PIDMoveArm(arm,ArmProfiledPID, 0.0),
        new RunShooter(shooter, intake, () -> 1.0, false,true),
        new ParallelCommandGroup(
          new GeneralTrajectories().toLeftBackNote(swerveSubsystem),
          // new PIDMoveArm(arm, 0.0),
          new RunIntake(intake, true, 1.0, true)
        ),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new PIDMoveArm(arm,ArmProfiledPID, 0.0),
        new RunShooter(shooter, intake, () -> 1.0, false,true)
      );
      break;

      case AmpScore:
       //Specific setpoints not finished, remove this line when tested and finished
      autoRoutine = new SequentialCommandGroup(
        // new InstantCommand(() -> limelight.resetLimelightPose()),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new GeneralTrajectories().toTag(swerveSubsystem),
        new PIDMoveArm(arm, ArmProfiledPID, 0.0),
        new RunShooter(shooter, intake, () -> 1.0, false,true)
      );
      break;

      case Driveout:
      autoRoutine = new GeneralTrajectories().Back(swerveSubsystem);
      break;

      case DoNothing:
      autoRoutine = new DoNothing();
  }
  return autoRoutine;
}
}

