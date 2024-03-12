// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Iterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Limelight;

public class ArmToShootingHeight extends Command {
  PIDMoveArm pidMoveArm;
  Limelight limelight;
  Map<Integer, double[]> distances = new HashMap<Integer, double[]>() {{
    put(0, new double[] {1,2,3,4,5});
    put(20, new double[] {1,2,3,4,5});
  }};
  Map<Integer, double[]> heights = new HashMap<Integer, double[]>() {{
    put(0, new double[] {1,2,3,4,5});
  }};
  double[] distancesFromKey;
  double[] heightsFromKey;
  double currentDistance;
  double currentAngle;
  double finalHeight;
  double divByZeroHandler = 0.0001;
  List<Double> calculatedHeight;
  List<Integer> keys;
  int index;
  Arm arm;
  ArmProfiledPID armProfiledPID;
  boolean done;

  public ArmToShootingHeight(PIDMoveArm pidMoveArm, Limelight limelight, Arm arm, ArmProfiledPID armProfiledPID) {
    this.pidMoveArm = pidMoveArm;
    this.arm = arm;
    this.armProfiledPID = armProfiledPID;
  }

  public void weightedCalculation(double currentValue, double dValueOne, double dValueTwo, double hValueOne, double hValueTwo) {
    calculatedHeight.add(
      ((( currentValue - dValueOne ) / ( dValueTwo - dValueOne + divByZeroHandler)) * hValueOne) + 
      ((( dValueTwo - currentValue ) / ( dValueTwo - dValueOne + divByZeroHandler)) * hValueTwo));
  }

  public void calculateHeight() {
    for (int i = 0; i < distancesFromKey.length; i++) {
      if (i != distancesFromKey.length-1) {
        if (currentDistance < distancesFromKey[i+1] && currentDistance > distancesFromKey[i]) {
          index = i;
          // takes the weighted average of the two heights that the current distance is between, depending on how close it is to each
          weightedCalculation(currentDistance, distancesFromKey[index], distancesFromKey[index+1], heightsFromKey[index], heightsFromKey[index+1]);
        }
      }
      else {
        index = i;
        weightedCalculation(currentDistance, distancesFromKey[index], distancesFromKey[0], heightsFromKey[index], heightsFromKey[0]);
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    currentDistance = Math.sqrt(Math.pow(limelight.targetpose.getFrontBackDistance(),2)+Math.pow(limelight.getLeftRightDistance(), 2));
    currentAngle = limelight.gettx();

    Iterator<Map.Entry<Integer, double[]>> distanceIterator = distances.entrySet().iterator();
    Map.Entry<Integer, double[]> current, next;

    while (distanceIterator.hasNext() || keys.isEmpty()) {
      current = distanceIterator.next();
      if (distanceIterator.hasNext()) {
        next = distanceIterator.next();
        if (currentAngle < current.getKey() && currentAngle > next.getKey()) {
          distancesFromKey = distances.get(current.getKey());
          heightsFromKey = heights.get(current.getKey());
          keys.add(current.getKey());
          keys.add(next.getKey());
          calculateHeight();
          weightedCalculation(currentAngle, keys.get(0), keys.get(1), calculatedHeight.get(0), calculatedHeight.get(1));
          finalHeight = calculatedHeight.get(2);
          }
        }
      else {
        distancesFromKey = distances.get(current.getKey());
        heightsFromKey = heights.get(current.getKey());
        calculateHeight();
        finalHeight = calculatedHeight.get(0);
      }
    }
    
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
    return done;
  }
}
