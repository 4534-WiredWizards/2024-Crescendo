// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.centerFourNote;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CommandConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.AutoRoutines.autoShoot;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnNote;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RotateByDegrees;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class redFourNoteCenter extends SequentialCommandGroup {

  /** Creates a new PlaceAndStation. */
  public redFourNoteCenter(
    Limelight limelight,
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    addCommands(
      new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter),
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redSpeakerNote, true),
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.autoIntakeSpeed,
          true
        )
          .withTimeout(3)
      ),
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter), // Fix the constructor cal
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redAmpNoteFour, true),
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.autoIntakeSpeed,
          true
        )
      ),
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter),
      new ParallelCommandGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory(swerve, AutoTrajectories.redSpeakerNoteFour, true),
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.autoIntakeSpeed,
          true
        )
      ),
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter) // Fix the constructor call
    );
  }
}
