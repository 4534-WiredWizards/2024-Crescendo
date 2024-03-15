// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.centerTwoNote;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class blueTwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new PlaceAndStation. */
  public blueTwoNoteCenter(Arm arm, ArmProfiledPID armProfiledPID, Intake intake, SwerveSubsystem swerve, Shooter shooter) {
  
    addCommands(
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, AutoTrajectories.blueSpeakerShoot, true),
        new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter)
      ),
      new ParallelCommandGroup(
        new PIDMoveArm(arm,armProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
        new FollowTrajectory(swerve, AutoTrajectories.blueSpeakerNote, true),
        new RunIntake(intake, true, .7, true)
      ),
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, AutoTrajectories.blueSpeakerShoot, true),
        new shootNoteWhenOnSub(arm, armProfiledPID, intake, swerve, shooter)
      )
    );    
  }

}



