// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class RunShooter extends Command {

  Shooter shooter;
  Supplier<Double> speed;
  double targetShooterVelocity;
  Intake Intake;
  boolean isPressed;
  int presses;
  boolean autostop;
  boolean autoIntake;
  private boolean PIDControl;

  /** Creates a new runShooter. */
  public RunShooter(
    Shooter shooter,
    Intake Intake,
    Supplier<Double> speed,
    boolean PIDControl,
    boolean autoStop,
    boolean autoIntake
  ) {
    this.shooter = shooter;
    this.speed = speed;
    this.Intake = Intake;
    this.PIDControl = PIDControl;
    this.autostop = autoStop;
    this.autoIntake = autoIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPressed = false;
    presses = 0;

    SmartDashboard.putNumber("Intake Velocity", 0);
    System.out.println(
      "Command '" +
      this.getName() +
      "' initialized at " +
      Timer.getFPGATimestamp()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Velocity", shooter.getSpeed());
    if (shooter.getSpeed() > 4500) {
      RobotContainer.leds.shooterStart();
    }
    if (PIDControl) {
      shooter.velocityPID(speed.get());
    } else {
      if (speed.get() > .1) {
        shooter.move(1.0);
      } else {
        shooter.move(0);
      }
      // shooter.move(speed.get());
    }

    // System.out.println(speed.get() * 5000);
    // TODO: Add logic for a parameter to not always auto run intake after reached velocity
    if (shooter.getSpeed() > 4800 && autoIntake) {
      System.out.println("Running Intake From Shooter");
      // Run RunShooter commmand from command
      // new RunIntake(Intake, true, .7, true);
      Intake.move(1);
      // TODO: TAKE THIS OUT,  blocks the main thread and the command scheduler
      try {
        Thread.sleep(200);
      } catch (InterruptedException e) {}
      // new WaitCommand(.2);
    }
    //logic is for isFinished condition, Will check for limit switch being pressed in, then out, twice before stopping.
    if (Intake.getIntakeStatus() && !isPressed) {
      presses += 1;
      isPressed = true;
    }
    if (!Intake.getIntakeStatus() && isPressed) {
      isPressed = false;
    }
  }

  // Called once the command ends or is interrupted

  @Override
  public void end(boolean interrupted) {
    System.out.println("Exited Shooter");
    shooter.move(0);
    if (autoIntake) {
      Intake.move(0);
    }
    System.out.println(
      "Command '" + this.getName() + "' ended at " + Timer.getFPGATimestamp()
    );
    // RobotContainer.leds.shooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autostop) {
      return (presses == 1 && !isPressed);
    } else {
      return false;
    }
  }
}
