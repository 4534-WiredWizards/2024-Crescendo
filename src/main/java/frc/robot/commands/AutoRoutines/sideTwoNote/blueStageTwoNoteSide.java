package frc.robot.commands.AutoRoutines.sideTwoNote;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.autonomous.AutoTrajectories;
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

public class blueStageTwoNoteSide extends ParallelCommandGroup {

  /** Creates a new PlaceAndStation. */
  public blueStageTwoNoteSide(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    Shooter shooter,
    SwerveSubsystem swerve
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
        new FollowTrajectory(swerve, AutoTrajectories.blueStageNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new RotateByDegrees(swerve, -27.6066),
        new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter)
      )
    );
  }
}
