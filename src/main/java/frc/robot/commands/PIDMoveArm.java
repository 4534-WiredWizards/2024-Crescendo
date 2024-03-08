// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Arm;

public class PIDMoveArm extends Command {
  ArmProfiledPID armProfiledPID;
  Arm arm;
  Double setpoint;
  /** Creates a new StateArm. */
  public PIDMoveArm(Arm arm, ArmProfiledPID armProfiledPID, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armProfiledPID = armProfiledPID;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armProfiledPID.setGoal(setpoint);
    armProfiledPID.enable();
     System.out.println("Start PIDMoveArm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ArmPIDMove Ended");
    arm.move(0);
    armProfiledPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // || (Math.abs(arm.getAbsolutePosition() - setpoint) < Units.degreesToRadians(2))
    if (armProfiledPID.atPIDGoal() ) {
      System.out.println("ArmPIDMove Finished");
      return true;
    } else {
      return false;
    }
    
  }

}
