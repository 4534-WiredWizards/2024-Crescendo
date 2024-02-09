// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class runClimb extends Command {
  /** Creates a new runClimb. */
  double TargetPos;
  Climb climb;
  double distanceFromTargetPos;
  double lastDistance;

  public runClimb(int climbPosition, Climb climb) {
    this.TargetPos = climbPosition;
    this.climb = climb;
    addRequirements(this.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceFromTargetPos = Math.abs(climb.getPosition() - Constants.CommandConstants.climbMidPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Parameter of integer between 1 and 3
    //1 = Stored while traversing
    //2 = prep for climb
    //3 = fully closed in climb

    if (TargetPos == 0) {
      climb.move(Constants.CommandConstants.climbWindSpeed);
    }
    else if (TargetPos < 0) {
      if (climb.getPosition() > TargetPos) {
        climb.move(Constants.CommandConstants.climbWindSpeed);
        distanceFromTargetPos -= lastDistance - climb.getPosition();
      }
      else {
        climb.move(Constants.CommandConstants.climbUnwindSpeed);
        distanceFromTargetPos -= climb.getPosition() - lastDistance;
      }

      lastDistance = climb.getPosition();
    }
    else {
      climb.move(0);
      distanceFromTargetPos = 0;
      System.out.println("Invalid climb command " + TargetPos);
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
    if (climb.getClimbStatus() && TargetPos == 0) {
      climb.EncoderReset();
      return true;
    }
    else {
      if (distanceFromTargetPos <= 0) {
        return true;
      }
      else {
        return false;
      }
    }
  }
}