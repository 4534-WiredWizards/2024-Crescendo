// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class spinShooterLowerArm extends ParallelDeadlineGroup {

  /** Creates a new shootNoteLowerArm. */
  public spinShooterLowerArm(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Intake intake,
    SwerveSubsystem swerve,
    Shooter shooter
  ) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new PIDMoveArm(
        arm,
        armProfiledPID,
        Units.degreesToRadians(CommandConstants.Arm.closeSpeaker),
        true
      )
    );
    addCommands(new RunShooter(shooter, intake, () -> 1.0, false, true, false));
  }
}
