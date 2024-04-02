// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.commands.CalculateArmAngle;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.PointToSpeaker;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class autoShoot extends SequentialCommandGroup {

  /** Creates a new PlaceAndStation. */
  public autoShoot(
    Limelight limelight,
    SwerveSubsystem swerve,
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    Shooter shooter
  ) {
    addCommands(
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new CalculateArmAngle(arm, armProfiledPID, limelight, true),
          new PointToSpeaker(limelight, swerve)
        ),
        new RunShooter(shooter, intake, () -> 1.0, false, false, false)
      ),
      new RunShooter(shooter, intake, () -> 1.0, false, true, true)
        .withTimeout(1)
    );
  }
}
