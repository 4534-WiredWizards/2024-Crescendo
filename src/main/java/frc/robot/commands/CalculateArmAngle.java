// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.CalcMap;
import frc.robot.subsystems.Limelight;

public class CalculateArmAngle extends Command {

  private final Arm arm;
  private final ArmProfiledPID armProfiledPID;
  private final Limelight limelight;
  // Values for current distance to target
  private double currentDistance = 0.0;
  private double FrontBackDistance = 0.0;
  private double LeftRightDistance = 0.0;

  private final double maxDistance = 9999.0;
  private final double[] shooterDistance = {
    0.00,
    5.00,
    10.00,
    20.00,
    30.00,
    maxDistance + 1.0,
  };
  private final double[] shooterAngle = { 0.30, 0.25, 0.20, 0.15, 0.10, 0.10 };
  private final double[] shooterSpeed = { 1.00, 1.00, 1.00, 1.00, 1.00, 1.00 };

  /** Creates a new LongShot command. */
  public CalculateArmAngle(
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Limelight limelight
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armProfiledPID = armProfiledPID;
    this.limelight = limelight;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Retrieve angle and speed for current distance
    double[] angleAndSpeed = ShooterAngleAndSpeed();
    // Set arm's angle to the calculated value
    armProfiledPID.setGoal(angleAndSpeed[0]);
    // Enable arm's profiled PID control
    armProfiledPID.enable();
    // Print debug message
    System.out.println("Start CaclulateArmAngle");

    // Set intial values for current distance to target
    currentDistance =
      Math.sqrt(
        // Math.pow(limelight.getFrontBackDistance(), 2) +
        Math.pow(limelight.getLeftRightDistance(), 2)
      );
    armProfiledPID.setGoal(ShooterAngleAndSpeed()[0]);
    armProfiledPID.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Print debug message
    System.out.println("CaclulateArmAngle Ended");
    // Stop arm movement
    arm.move(0);
    // Disable arm's profiled PID control
    armProfiledPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Add your finish condition here
    return armProfiledPID.atPIDGoal();
  }

  // Calculates shooter angle and speed based on current distance to target
  private double[] ShooterAngleAndSpeed() {
    // Get current distance to target
    int index = 0;
    double blend;
    double[] results = new double[2];

    // Pin current distance to valid range
    currentDistance = (currentDistance < 0) ? 0.0 : currentDistance;
    currentDistance =
      (currentDistance >= maxDistance) ? maxDistance : currentDistance;

    // Find the last distance in the table that is <= the current distance
    while (currentDistance > shooterDistance[index]) {
      index++;
    }

    // Calculate blend factor between two closest distances
    blend =
      (currentDistance - shooterDistance[index]) /
      (shooterDistance[index + 1] - shooterDistance[index]);

    // Blend angle and speed values between two closest distances
    results[0] =
      shooterAngle[index] * (1.0 - blend) + shooterAngle[index + 1] * blend;
    results[1] =
      shooterSpeed[index] * (1.0 - blend) + shooterSpeed[index + 1] * blend;

    return results;
  }
}
