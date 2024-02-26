// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTagPositions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class PointToSpeaker extends Command {

  SwerveSubsystem swerveSubsystem;
  Limelight limelight;
  PIDController thetaController;
  Pose2d position;
  double desiredTheta;
  double botRot;
  double SpeedRadiansPerSecond;

  /** Creates a new PointToSpeaker. */
  public PointToSpeaker(
    SwerveSubsystem swerveSubsystem,
    Limelight limelight
    ) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.position = swerveSubsystem.getPose();
    
    thetaController = new PIDController(2, 0.5, 0);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.resetLimelightBotPose();
    double botX = position.getX();
    double botY = position.getY();
    botRot = Units.degreesToRadians(swerveSubsystem.getHeading());
    System.out.println("botRot " + botRot);
    thetaController.setTolerance(Units.degreesToRadians(5));

    if (AprilTagPositions.Tag4_y - botY == 0) {
      desiredTheta = 0;
    }
    else if (AprilTagPositions.Tag4_x - botX == 0) {
      desiredTheta = (3 * Math.PI) / 2;
    }
    else {
      desiredTheta = Math.atan(botY - AprilTagPositions.Tag4_y / botX - AprilTagPositions.Tag4_x);
    desiredTheta += Math.PI;
    System.out.println("desiredTheta " + desiredTheta);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = swerveSubsystem.getPose();
    botRot = Units.degreesToRadians(swerveSubsystem.getHeading());
    SpeedRadiansPerSecond = thetaController.calculate(botRot, desiredTheta);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    0, 0, SpeedRadiansPerSecond, swerveSubsystem.getRotation2d());
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return thetaController.atSetpoint();
    }
}
