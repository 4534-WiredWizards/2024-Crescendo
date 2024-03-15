// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;


// new SequentialCommandGroup(
//             new ParallelCommandGroup(
//               new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
//               shootNoteWhenOnSub()     
//             ),
//             new ParallelCommandGroup(
//               new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
//               new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerNote, true),
//               new RunIntake(intake, true, .7, true)
//             ),
//             new ParallelCommandGroup(
//               new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
//              shootNoteWhenOnSub()
//             )
//             // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
//         );
//Covert to a command group

public class blueTwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new PlaceAndStation. */
  public blueTwoNoteCenter(Arm arm) {
    addCommands(
      new ParallelCommandGroup(
        new FollowTrajectory(SwerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
        new shootNoteWhenOnSub()     
      ),
      new ParallelCommandGroup(
        new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
        new FollowTrajectory(AutoTrajectories.blueSpeakerNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new FollowTrajectory(AutoTrajectories.blueSpeakerShoot, true),
        new shootNoteWhenOnSub()
      )
    );
  }

}



