// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class FocusShoot extends Command {

  //Command that works similar to PointToSpeaker.java, but allows continued input of x and y values the robot while its rotating to the target
  //Also moves the arm to the target height using   new CalculateArmAngle(arm, armProfiledPID, limelight, false)
  private final Supplier<Double> xSpdFunction, ySpdFunction, throttle;
  private double throttleadjusted;
  Limelight limelight;
  SwerveSubsystem swerve;
  CalculateArmAngle calculateArmAngle;
  ArmProfiledPID armProfiledPID;
  Arm arm;
  PIDController PIDController;
  Double distance;
  private final SlewRateLimiter xLimiter, yLimiter;

  /** Creates a new PointToSpeaker2. */
  public FocusShoot(
    Limelight limelight,
    SwerveSubsystem swerve,
    Arm arm,
    ArmProfiledPID armProfiledPID,
    Supplier<Double> xSpdFunction,
    Supplier<Double> ySpdFunction,
    Supplier<Double> throttle
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.throttle = throttle;
    this.xLimiter =
      new SlewRateLimiter(
        DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
      );
    this.yLimiter =
      new SlewRateLimiter(
        DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
      );

    // Constant values
    calculateArmAngle =
      new CalculateArmAngle(arm, armProfiledPID, limelight, false);
    PIDController = new PIDController(0.2, 0, 0.4);
    PIDController.setTolerance(2, 1);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    distance = swerve.getHeading() - (limelight.gettx() + 4);
    if (limelight.gettx() == 0) {
      distance = swerve.getHeading();
    }

    // 1. Get real-time joystick inputs
    throttleadjusted = throttle.get() * -.25 + .75;
    double xSpeed = xSpdFunction.get() * throttleadjusted;
    double ySpeed = ySpdFunction.get() * throttleadjusted;
    double turningSpeed = PIDController.calculate(
      swerve.getHeading(),
      distance
    );
    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed =
      xLimiter.calculate(xSpeed) *
      DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed =
      yLimiter.calculate(ySpeed) *
      DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    // 4. Construct desired chassis speeds && move arm to target height
    calculateArmAngle.execute(); // Move arm to target height
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed,
      ySpeed,
      turningSpeed * .5,
      swerve.getRotation2d()
    );
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      chassisSpeeds
    );
    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    System.out.println(
      "Command '" + this.getName() + "' ended at " + Timer.getFPGATimestamp()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
