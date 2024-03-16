// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Targetpose;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateByDegrees extends Command {

  Limelight limelight;
  SwerveSubsystem swerve;
  PIDController PIDController;
  Double rotationSpeed;
  Double distance;
  Double degrees;

  /** Creates a new PointToSpeaker2. */
  public RotateByDegrees(SwerveSubsystem swerve, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.degrees = degrees;
    PIDController = new PIDController(0.3, .75, 0.02);
    PIDController.setTolerance(2, 1);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = swerve.getHeading() - degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationSpeed = PIDController.calculate(swerve.getHeading(), distance);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      0,
      0,
      rotationSpeed * .5,
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint();
  }
}
