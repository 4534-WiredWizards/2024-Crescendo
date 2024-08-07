// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

  Shooter shooter;
  Intake Intake;
  Boolean IntakeStatus;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Intake Intake) {
    this.shooter = shooter;
    this.Intake = Intake;

    addRequirements(this.Intake, this.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Shoot(
    Shooter shooter2,
    frc.robot.subsystems.Intake intake2,
    Object object,
    boolean b,
    boolean c,
    boolean d
  ) {
    //TODO Auto-generated constructor stub
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.IntakeStatus = Intake.getIntakeStatus();
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
    shooter.move(1);
    if ((shooter.getSpeed() > 4750)) {
      // RobotContainer.leds.shooterStart();
      Intake.move(1.0);
      try {
        Thread.sleep(200);
      } catch (InterruptedException e) {}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Exited Shooter");
    // RobotContainer.leds.shooterStop();
    shooter.move(0);
    Intake.move(0);
    System.out.println(
      "Command '" + this.getName() + "' ended at " + Timer.getFPGATimestamp()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!IntakeStatus == Intake.getIntakeStatus());
  }
}
