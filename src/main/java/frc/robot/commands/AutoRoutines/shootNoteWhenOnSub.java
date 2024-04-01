// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RampUpShooter;
import frc.robot.commands.RunShooter;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootNoteWhenOnSub extends SequentialCommandGroup {

  /** Creates a new shootNoteWhenOnSub. */
  public shootNoteWhenOnSub(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    addCommands(
      new ParallelDeadlineGroup(
        new PIDMoveArm(
          arm,
          armProfiledPID,
          Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
          true
        ),
        new RampUpShooter(shooter)
      ),
      new Shoot(shooter, intake)
      // new RunShooter(shooter, intake, () -> 1.0, false, true, true)
    );
  }
}
