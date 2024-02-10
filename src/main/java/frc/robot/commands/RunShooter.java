// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class RunShooter extends Command {
  Shooter shooter;
  Supplier<Double> speed;
  Intake Intake;
  boolean isPressed;
  int presses;
  boolean autostop;
  /** Creates a new runShooter. */
  public  RunShooter(Shooter shooter, Intake Intake, Supplier<Double> speed, boolean autoStop) {
    this.shooter = shooter;
    this.speed = speed;
    this.Intake = Intake;
    this.autostop = autoStop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  isPressed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.move(speed.get());
    if(shooter.getSpeed() >  3000){
    Intake.move(speed.get());
    }
    //logic is for isFinished condition, Will check for limit switch being pressed in, then out, twice before stopping.
    if(Intake.getIntakeStatus() && !isPressed){
      presses += 1;
      isPressed = true;
    }
    if(!Intake.getIntakeStatus() && isPressed) {
      isPressed = false;
    }

  }

  // Called once the command ends or is interrupted

  @Override
  public void end(boolean interrupted) {
    shooter.move(0);
    Intake.move(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(autostop){
      return(presses > 1 & !isPressed);
    }
    else{
      return false;
    }
  }
}
