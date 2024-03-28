// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CommandConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class intake extends ParallelCommandGroup {

  /** Creates a new PlaceAndStation. */
  public intake(Arm arm, ArmProfiledPID armProfiledPID, Intake intake) {
    addCommands(
      new PIDMoveArm(
        arm,
        armProfiledPID,
        Units.degreesToRadians(CommandConstants.Arm.intake),
        true
      ),
      new RunIntake(
        intake,
        true,
        Constants.CommandConstants.Intake.autoIntakeSpeed,
        true
      )
    );
  }
}
