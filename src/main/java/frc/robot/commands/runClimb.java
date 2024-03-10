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
  boolean windClimb;

  public runClimb(int climbPosition, Climb climb) {
    this.TargetPos = climbPosition;
    this.climb = climb;
    addRequirements(this.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // distanceFromTargetPos = Math.abs(climb.getPosition() - Constants.CommandConstants.climbMidPos);
    if (climb.getPosition() > TargetPos){
      this.windClimb=true;
    } else {
      this.windClimb=false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.out.println("Climb Status->"+climb.getClimbStatus());
    if (TargetPos == 0) {
      // if  (!climb.getClimbStatus()) {
        climb.move(Constants.CommandConstants.Climb.windSpeed);
      // }
      // else{
      //   climb.EncoderReset();
      // }
    } else if (TargetPos > 0) {
      
      if (windClimb) {
        climb.move(Constants.CommandConstants.Climb.windSpeed);
        // distanceFromTargetPos = TargetPos-climb.getPosition();
      } else {
        climb.move(Constants.CommandConstants.Climb.unwindSpeed);
        // distanceFromTargetPos = TargetPos-climb.getPosition();
      }
    } else {
      climb.move(0);
      distanceFromTargetPos = 0;
      // System.out.println("Invalid climb command " + TargetPos);
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
    } else {
      if(windClimb && climb.getPosition()<TargetPos){
        return true;
      } else if (!windClimb && climb.getPosition()>TargetPos){
        return true;
      } else {
        return false;
      }
    }
    // return false;
  }
}
