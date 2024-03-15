// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootNoteWhenOnSub extends SequentialCommandGroup {
  /** Creates a new shootNoteWhenOnSub. */
  public shootNoteWhenOnSub(Arm arm, ArmProfiledPID armProfiledPID, Intake intake, SwerveSubsystem swerve, frc.robot.subsystems.Shooter shooter) {

    addCommands(
      new shootNoteLowerArm(arm, armProfiledPID, intake, swerve, shooter),
      new RunShooter(shooter, intake, () -> 1.0, false,true, true)
    );
  }
}
