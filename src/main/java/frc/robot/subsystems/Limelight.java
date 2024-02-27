// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  double cameraPose[];
  public SwerveSubsystem swerve;
  public Limelight(SwerveSubsystem givenSwerve) {
    swerve=givenSwerve;
  }

  static final int kX = 0;
  static final int kY = 1; 
  static final int kZ = 2;
  static final int kbpRoll = 3;
  static final int kbpPitch = 4;
  static final int kbpYaw = 5;

  static final int ktpPitch = 3;
  static final int ktpYaw = 4;
  static final int ktpRoll = 5;


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Botpose botpose = new Botpose();
  public Targetpose targetpose = new Targetpose();

  public class Botpose {
    public double getXDistance() {
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Botpose X Distance", cameraPose[kX]);
      return cameraPose[kX];
    }

    public double getYDistance() {
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Botpose Y Distance", cameraPose[kY]);
      return cameraPose[kY];
    }

    public double getThetaDegreesFeild() {
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Botpose Theta Degs", cameraPose[kbpYaw]);
      return cameraPose[kbpYaw];
    }
  }

  public class Targetpose {
    public double getFrontBackDistance() {
      // Further the april tag, larger the int
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Targetpose Z Distance", cameraPose[kZ]);
      return cameraPose[kZ];
    }

    public double getLeftRightDistance() {
      // April tag on the left is negitve
      // April tag on the right is positve
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Targetpose X Distance", cameraPose[kX]);
      return cameraPose[kX];
    }

    public double getThetaDegrees() {
      cameraPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Targetpose Theta Degs", cameraPose[ktpYaw]);
      return cameraPose[ktpYaw];
    }

    public void resetLimelightPose(){
      swerve.resetOdometry(new Pose2d(targetpose.getFrontBackDistance(), targetpose.getLeftRightDistance(), Rotation2d.fromDegrees(targetpose.getThetaDegrees())));
    }
  }

  public void resetLimelightTargetPose(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // System.out.println(targetpose.getFrontBackDistance());
    // System.out.println(targetpose.getLeftRightDistance());
    // System.out.println(targetpose.getThetaDegrees());
    if (!(targetpose.getFrontBackDistance() == 0.0 && targetpose.getLeftRightDistance() == 0.0 && targetpose.getThetaDegrees() == 0.0)) {
      swerve.resetOdometry(new Pose2d(targetpose.getFrontBackDistance(), targetpose.getLeftRightDistance(), Rotation2d.fromDegrees(-targetpose.getThetaDegrees())));
    }
    
  }

  public void resetLimelightBotPose(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    System.out.println("Bp X: "+botpose.getXDistance());
    System.out.println("Bp Y:"+botpose.getYDistance());
    System.out.println("Bp Theta:"+botpose.getThetaDegreesFeild());
    swerve.resetOdometry(new Pose2d(botpose.getXDistance(), botpose.getYDistance(), Rotation2d.fromDegrees(-botpose.getThetaDegreesFeild())));
    
  }
}
