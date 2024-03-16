// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.centerFourNote;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnNote;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RotateByDegrees;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class blueFourNoteCenter extends SequentialCommandGroup {

  /** Creates a new PlaceAndStation. */
  public blueFourNoteCenter(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    addCommands(
      new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter),
      new RunShooter(shooter, intake, () -> 1.0, false, true, true)
        .withTimeout(1),
      new ParallelCommandGroup( // Arm Down - Angle back
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.blueStageNoteFour, true),
        new RunIntake(intake, true, .7, true)
      ),
      new shootNoteWhenOnNote(arm, armProfiledPID, intake, swerve, shooter)
    );
  }
}
