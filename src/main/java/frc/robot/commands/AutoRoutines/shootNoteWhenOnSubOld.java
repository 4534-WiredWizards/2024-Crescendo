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

public class shootNoteWhenOnSubOld extends ParallelCommandGroup {
  /** Creates a new PlaceAndStation. */
  public shootNoteWhenOnSub(Arm arm, ArmProfiledPID armProfiledPID, Intake intake, SwerveSubsystem swerve, frc.robot.subsystems.Shooter shooter) {
    addCommands(
      new PIDMoveArm(arm,armProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
      new SequentialCommandGroup(
        new DoNothing().withTimeout(1.2), //Wait a for robot to drive back to shooting position
        new RunShooter(shooter, intake, () -> 1.0, false,true, true)
      )
    );
  }

}



