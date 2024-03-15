package frc.robot.commands.AutoRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class blueStageTwoNoteSide extends ParallelCommandGroup {
  /** Creates a new PlaceAndStation. */
  public blueStageTwoNoteSide(Arm arm, ArmProfiledPID armProfiledPID, Intake intake, SwerveSubsystem swerve, frc.robot.subsystems.Shooter shooter) {
   new SequentialCommandGroup(
          shootNoteWhenOnSub(),    
          new ParallelCommandGroup(
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueStageNote, true),
            new RunIntake(intake, true, .7, true)
          ),
          new ParallelCommandGroup(
            new RotateByDegrees(swerveSubsystem, -27.6066),
            shootNoteWhenOnNote()
          )
          );
  }

}



