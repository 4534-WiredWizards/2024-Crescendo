// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

  /** Creates a new Limelight. */
  NetworkTable table;
  double cameraPose[];
  public SwerveSubsystem swerve;

  public Limelight(SwerveSubsystem givenSwerve) {
    swerve = givenSwerve;
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
      cameraPose =
        NetworkTableInstance
          .getDefault()
          .getTable("limelight")
          .getEntry("botpose")
          .getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Botpose X Distance", cameraPose[kX]);
      return cameraPose[kX];
    }

    public double getYDistance() {
      cameraPose =
        NetworkTableInstance
          .getDefault()
          .getTable("limelight")
          .getEntry("botpose")
          .getDoubleArray(new double[6]);
      SmartDashboard.putNumber("Limelight Botpose Y Distance", cameraPose[kY]);
      return cameraPose[kY];
    }

    public double getThetaDegreesField() {
      cameraPose =
        NetworkTableInstance
          .getDefault()
          .getTable("limelight")
          .getEntry("botpose")
          .getDoubleArray(new double[6]);
      SmartDashboard.putNumber(
        "Limelight Botpose Theta Degs",
        cameraPose[kbpYaw]
      );
      return cameraPose[kbpYaw];
    }
  }

  public class Targetpose {

    public double getFrontBackDistance() {
      // Further the april tag, larger the int
      cameraPose =
        NetworkTableInstance
          .getDefault()
          .getTable("limelight")
          .getEntry("camerapose_targetspace")
          .getDoubleArray(new double[6]);
      SmartDashboard.putNumber(
        "Limelight Targetpose Z Distance",
        cameraPose[kZ]
      );
      return cameraPose[kZ];
    }

    public double getThetaDegrees() {
      cameraPose =
        NetworkTableInstance
          .getDefault()
          .getTable("limelight")
          .getEntry("camerapose_targetspace")
          .getDoubleArray(new double[6]);
      SmartDashboard.putNumber(
        "Limelight Targetpose Theta Degs",
        cameraPose[ktpYaw]
      );
      return cameraPose[ktpYaw];
    }

    public void resetLimelightPose() {
      swerve.resetOdometry(
        new Pose2d(
          targetpose.getFrontBackDistance(),
          getLeftRightDistance(),
          Rotation2d.fromDegrees(targetpose.getThetaDegrees())
        )
      );
    }
  }

  public void resetLimelightTargetPose() {
    // Do not use for
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(1);
    // System.out.println(targetpose.getFrontBackDistance());
    // System.out.println(targetpose.getLeftRightDistance());
    // System.out.println(targetpose.getThetaDegrees());
    if (
      !(
        targetpose.getFrontBackDistance() == 0.0 &&
        getLeftRightDistance() == 0.0 &&
        targetpose.getThetaDegrees() == 0.0
      )
    ) {
      swerve.resetOdometry(
        new Pose2d(
          targetpose.getFrontBackDistance(),
          getLeftRightDistance(),
          Rotation2d.fromDegrees(-targetpose.getThetaDegrees())
        )
      );
    }
  }

  public boolean resetLimelightBotPose(
    Double botX,
    Double botY,
    Double rotation
  ) {
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(1);
    // Double botX = botX;
    // Double botY = botY;
    // Double rotation = botpose.getThetaDegreesField();
    System.out.println("Bp X: " + botX);
    System.out.println("Bp Y:" + botY);
    System.out.println("Bp Rotation:" + rotation);
    if (botX < 0) {
      // On blue side of field, add 180 to theta
      rotation -= 180;
    } else if (botX > 0) {
      // On red side of field, add 180 to theta
      rotation += 180;
    }
    swerve.resetOdometry(
      new Pose2d(botX, botY, Rotation2d.fromDegrees(rotation))
    );
    if (botX != 0) {
      return true;
    } else {
      System.err.println("NO VALUE FOR TAG'S");
      return false;
    }
  }

  public double gettx() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getLeftRightDistance() {
    // April tag on the left is negitve
    // April tag on the right is positve

    cameraPose =
      NetworkTableInstance
        .getDefault()
        .getTable("limelight")
        .getEntry("camerapose_targetspace")
        .getDoubleArray(new double[6]);

    SmartDashboard.putNumber("Limelight Targetpose X Distance", cameraPose[kX]);
    return cameraPose[kX];
  }
}
