// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Targetpose;
import frc.robot.subsystems.SwerveSubsystem;

public class PointToSpeaker2 extends Command {

  Limelight limelight;
  SwerveSubsystem swerve;
  PIDController PIDController;
  Double rotationSpeed;
  Double distance;

  /** Creates a new PointToSpeaker2. */
  public PointToSpeaker2(Limelight limelight, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    PIDController = new PIDController(0.3, .75, 0.02);
    PIDController.setTolerance(2, 1);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = swerve.getHeading() - (limelight.gettx() + 4);
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
    rotationSpeed = PIDController.calculate(swerve.getHeading(), distance);
    // System.out.println(limelight.gettx());
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
    System.out.println(
      "Command '" + this.getName() + "' ended at " + Timer.getFPGATimestamp()
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDController.atSetpoint() && (limelight.gettx() < 5);
  }
}
