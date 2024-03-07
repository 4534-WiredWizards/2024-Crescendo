// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagPositions;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

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
    
    
    thetaController = new PIDController(1, 0.5, 0);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.resetLimelightBotPose();
    this.position = swerveSubsystem.getPose();
    double distanceX = position.getX() - AprilTagPositions.Tag4_x;
    double distanceY = position.getY() - AprilTagPositions.Tag4_y;
    if (distanceX == 0) {distanceX += 0.001;}
    if (distanceY == 0) {distanceY += 0.001;}
    double ratio = (distanceY / distanceX);
    botRot = Units.degreesToRadians(swerveSubsystem.getHeading());
    botRot = (botRot + Math.PI * 2) % Math.PI * 2;
    System.out.println("botRot " + botRot);
    thetaController.setTolerance(Units.degreesToRadians(5));
    desiredTheta = Math.atan(ratio);
    desiredTheta = (desiredTheta + Math.PI * 2) % Math.PI * 2;
    System.out.println("desiredTheta 1 " + desiredTheta);
    if (distanceX > 0) {
      desiredTheta += Math.PI;
      desiredTheta = (desiredTheta + Math.PI * 2) % Math.PI * 2;}
    System.out.println("desiredTheta 2 " + desiredTheta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = swerveSubsystem.getPose();
    botRot = Units.degreesToRadians(swerveSubsystem.getHeading());
    botRot = (botRot + Math.PI * 2) % Math.PI * 2;
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
