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
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class blueFourNoteCenter extends SequentialCommandGroup {

  /** Creates a new PlaceAndStation. */
  public blueFourNoteCenter(
    Limelight limelight,
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    addCommands(
      new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter), // Shoot pre loaded note
      new ParallelCommandGroup( // In Paaallel - Arm Down - Drive to first note - Start intake
        new PIDMoveArm( // Arm Down to intake position
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new SequentialCommandGroup(
          new DoNothing().withTimeout(.2),
          new FollowTrajectory(swerve, AutoTrajectories.blueSpeakerNote, true)
        ),
        new RunIntake(
          intake,
          true,
          Constants.CommandConstants.Intake.autoIntakeSpeed,
          true
        )
          .withTimeout(3) // Start intake
      ),
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter), // Fix the constructor call
      new ParallelCommandGroup( // In Parallel - Arm Down - Drive to second note - Start intake
        new PIDMoveArm( // Arm Down to intake position
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory( //Drive in arc from stage note to speaker note
          swerve,
          AutoTrajectories.blueAmpNoteFour,
          true
        ),
        new RunIntake( // Start intake
          intake,
          true,
          Constants.CommandConstants.Intake.autoIntakeSpeed,
          true
        )
      ),
      new autoShoot(limelight, swerve, arm, armProfiledPID, intake, shooter),
      new ParallelCommandGroup( // In Parallel - Arm Down - Drive to second note - Start intake
        new PIDMoveArm( // Arm Down to intake position
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.intake),
          true
        ),
        new FollowTrajectory( //Drive in arc from stage note to speaker note
          swerve,
          AutoTrajectories.blueStageNoteFour,
          true
        ),
        new RunIntake( // Start intake
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
