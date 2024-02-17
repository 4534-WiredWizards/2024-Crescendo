// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;


public class MoveArm extends Command {
  /** Creates a new ControlShooterArm. */
  Arm Arm;
  Double speed;
  public 
  MoveArm(Arm Arm, Double speed) {
    this.Arm = Arm;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.Arm);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Move Arm Called");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.move(speed);
    // System.out.println("Moving Arm Spd:"+speed);

    // Test Low and high limits then implement limit switch logic
    //
    // if(speed < 0){
    //   if (Arm.getAbsolutePosition() > [low]) {
    //     shooterArm.move(speed);
    //   else{
    //     shooterArm.move(0)
    //   }
    //   }
    // }
    // else if(speed > 0){
    //   if (Arm.getAbsolutePosition() < [high]) {
    //     shooterArm.move(speed);
    //   else{
    //     shooterArm.move(0)
    //   }
    //   }
    // }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}