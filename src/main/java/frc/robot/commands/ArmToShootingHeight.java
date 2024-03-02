// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Limelight;

public class ArmToShootingHeight extends Command {
  PIDMoveArm pidMoveArm;
  Limelight limelight;
  double[] Distances = {1,2,3,4,5};
  double[] Heights = {1,2,3,4,5};
  double currentDistance;
  double calculatedHeight;
  int index;
  Arm arm;
  ArmProfiledPID armProfiledPID;
  int done;

  public ArmToShootingHeight(PIDMoveArm pidMoveArm, Limelight limelight, Arm arm, ArmProfiledPID armProfiledPID) {
    this.pidMoveArm = pidMoveArm;
    this.arm = arm;
    this.armProfiledPID = armProfiledPID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = 0;
    currentDistance = Math.sqrt(Math.pow(limelight.targetpose.getFrontBackDistance(),2)+Math.pow(limelight.getLeftRightDistance(), 2));
    for (int i = 0; i < Distances.length; i++) {
      if (currentDistance < Distances[i+1] && currentDistance > Distances[i]) {
        index = i;
      }
      
    } 
    calculatedHeight = (((currentDistance - Distances[index])/(Distances[index+1]-Distances[index])) * Heights[index]) + (((Distances[index + 1] - currentDistance)/(Distances[index+1]-Distances[index])) * Heights[index+1]);
    new PIDMoveArm(arm, armProfiledPID, calculatedHeight);
    done = 1;
    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (done == 1);
  }
}
