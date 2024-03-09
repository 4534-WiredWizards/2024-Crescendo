// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class manualClimb extends Command {
  /** Creates a new runClimb. */
  Climb climb;
  Boolean windBoolean;

  public manualClimb(boolean wind,Climb climb) {
    this.climb = climb;
    this.windBoolean = wind;
    addRequirements(this.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Climb Status->"+climb.getClimbStatus());
    if (windBoolean) {
        climb.move(Constants.CommandConstants.Climb.windSpeed);
    } else {
        climb.move(Constants.CommandConstants.Climb.unwindSpeed);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}