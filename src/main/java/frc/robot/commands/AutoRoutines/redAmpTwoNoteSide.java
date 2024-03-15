// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import

public class redAmpTwoNoteSide extends ParallelCommandGroup {
  /** Creates a new PlaceAndStation. */
  public redAmpTwoNoteSide(Arm arm, ArmProfiledPID armProfiledPID, Intake intake, SwerveSubsystem swerve, frc.robot.subsystems.Shooter shooter) {
    new SequentialCommandGroup(
          shootNoteWhenOnSub(),    
          new ParallelCommandGroup(
            new PIDMoveArm(arm, armProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
            new FollowTrajectory(swerve, AutoTrajectories.redAmpNote, true),
            new RunIntake(intake, true, .7, true)
          ),
          new ParallelCommandGroup(
            new RotateByDegrees(swerveSubsystem, -28.2685),
            shootNoteWhenOnNote()
          )
          );
  }

}



