// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.centerFourNote;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnNote;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RotateByDegrees;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class redFourNoteCenter extends ParallelCommandGroup {

  /** Creates a new PlaceAndStation. */
  public redFourNoteCenter(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    new SequentialCommandGroup(
      new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter),
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redAmpFourNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new RotateByDegrees(swerve, -28.2685),
        new shootNoteWhenOnNote(arm, armProfiledPID, intake, swerve, shooter),
        new RotateByDegrees(swerve, 118.2685)
      ),
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redSpeakerFourNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new RotateByDegrees(swerve, -90),
        new shootNoteWhenOnNote(arm, armProfiledPID, intake, swerve, shooter),
        new RotateByDegrees(swerve, 90)
      ),
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redStageFourNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new RotateByDegrees(swerve, -61.7315),
        new shootNoteWhenOnNote(arm, armProfiledPID, intake, swerve, shooter)
      )
    );
  }
}
