// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class runClimb extends Command {
  /** Creates a new runClimb. */
  int climbCmd;
  Climb climb;
  boolean isWinding = false;

  public runClimb(int climbPosition, Climb climb) {
    this.climbCmd = climbPosition;
    this.climb = climb;
    addRequirements(this.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Parameter of integer between 1 and 3
    //1 = Stored while traversing
    //2 = prep for climb
    //3 = fully closed in climb

    if (climbCmd == Constants.CommandConstants.lowestClimbCmd) {
      climb.move(Constants.CommandConstants.climbWindSpeed);
    }
    else if (climbCmd == Constants.CommandConstants.middleClimbCmd) {
      if (climb.getPosition() > Constants.CommandConstants.climbMidPos) {
        climb.move(Constants.CommandConstants.climbWindSpeed);
        isWinding = true;
      }
      else {
        climb.move(Constants.CommandConstants.climbUnwindSpeed);
        isWinding = false;
      }
    }
    else if (climbCmd == Constants.CommandConstants.highestClimbCmd) {
      climb.move(Constants.CommandConstants.climbUnwindSpeed);
    }
    else {
      climb.move(0);
      System.out.println("Invalid climb command " + climbCmd);
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
    if (climb.getForwardLimitSwitch() && climbCmd == Constants.CommandConstants.lowestClimbCmd) {
      return true;
    }
    else if (climb.getPosition() >= Constants.CommandConstants.climbHighPos) {
      return true;
    }
    else if (climbCmd == Constants.CommandConstants.middleClimbCmd) {
      if (isWinding) {
        if (climb.getPosition() <= Constants.CommandConstants.climbMidPos) {
          return true;
        }
        else {
          return false;
        }
      }
      else {
        if (climb.getPosition() >= Constants.CommandConstants.climbMidPos) {
          return true;
        }
        else {
          return false;
        }
      }
    }
    else {
      return false;
    }
  }
}
