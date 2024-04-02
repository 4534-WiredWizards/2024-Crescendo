// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.CalculateArmAngle;
import frc.robot.commands.PointToSpeaker;
import frc.robot.commands.ShootOnPID;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class autoShoot extends ParallelDeadlineGroup {

  /** Creates a new PlaceAndStation. */
  public autoShoot(
    Limelight limelight,
    SwerveSubsystem swerve,
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    Shooter shooter
  ) {
    super(new ShootOnPID(shooter, intake, armProfiledPID));
    addCommands(
      new PointToSpeaker(limelight, swerve),
      new CalculateArmAngle(arm, armProfiledPID, limelight, false)
    );
  }
}
