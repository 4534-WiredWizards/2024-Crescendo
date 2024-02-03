// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class PIDMoveArm extends Command {
  Arm arm;
  PIDController armpid;
  Double setpoint;
  /** Creates a new StateArm. */
  public PIDMoveArm(Arm arm, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armpid = new PIDController(.05,.01,.01);
    armpid.setTolerance(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.move(armpid.calculate(arm.getAbsolutePosition(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armpid.atSetpoint();
  }
}
