package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.AutoChooser.AllianceColor;

public class SwerveSubsystem extends SubsystemBase {

  private SendableChooser<AllianceColor> allianceColorChooser;

  public final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveMotorReversed,
    DriveConstants.kFrontLeftTurningMotorReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
    "Front Left"
  );

  public final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveMotorReversed,
    DriveConstants.kFrontRightTurningMotorReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
    "Front Right"
  );

  public final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveMotorReversed,
    DriveConstants.kBackLeftTurningMotorReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
    "Back Left"
  );

  public final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveMotorReversed,
    DriveConstants.kBackRightTurningMotorReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
    "Back Right"
  );

  // AHRS is NavX gryo module
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    new Rotation2d(0),
    getModulePositions()
  );

  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    })
      .start();
    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      AutoConstants.AutoHolonomicPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        AllianceColor selectedAllianceColor = (AllianceColor) allianceColorChooser.getSelected();
        if (selectedAllianceColor != null) {
          return selectedAllianceColor == AllianceColor.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  // Get relative robot speed from ChassisSpeeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    );
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      speeds
    );
    setModuleStates(moduleStates);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return (
      (DriveConstants.kGyroInverted ? -1.0 : 1.0) *
      Math.IEEEremainder((gyro.getAngle() * ModuleConstants.upDOG), 360)
    );
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };

    return positions;
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    odometer.update(getRotation2d(), getModulePositions());
    SmartDashboard.putNumber("Robot Heading", getHeading());
    // SmartDashboard.putString(
    //   "Robot Location",
    //   getPose().getTranslation().toString()
    // );
    // SmartDashboard.putNumber(
    //   "Back Right angle",
    //   (Units.radiansToDegrees(backRight.getAbsoluteEncoderRad()))
    // );
    // SmartDashboard.putNumber(
    //   "Back Left angle",
    //   (Units.radiansToDegrees(backLeft.getAbsoluteEncoderRad()))
    // );
    // SmartDashboard.putNumber(
    //   "Front Right angle",
    //   (Units.radiansToDegrees(frontRight.getAbsoluteEncoderRad()))
    // );
    // SmartDashboard.putNumber(
    //   "Front Left angle",
    //   (Units.radiansToDegrees(frontLeft.getAbsoluteEncoderRad()))
    // );

    // SmartDashboard.putString(
    //   "Back Right position",
    //   (backRight.getPosition().toString())
    // );
    // SmartDashboard.putString(
    //   "Back Left position",
    //   (backLeft.getPosition().toString())
    // );
    // SmartDashboard.putString(
    //   "Front Right position",
    //   (frontRight.getPosition().toString())
    // );
    // SmartDashboard.putString(
    //   "Front Left position",
    //   (frontLeft.getPosition().toString())
    // );

    // // Raw encoder values with no offset
    // SmartDashboard.putNumber(
    //   "Raw Back Right angle",
    //   (Units.radiansToDegrees(backRight.getRawEncoderValue()))
    // );
    // SmartDashboard.putNumber(
    //   "Raw Back Left angle",
    //   (Units.radiansToDegrees(backLeft.getRawEncoderValue()))
    // );
    // SmartDashboard.putNumber(
    //   "Raw Front Right angle",
    //   (Units.radiansToDegrees(frontRight.getRawEncoderValue()))
    // );
    // SmartDashboard.putNumber(
    //   "Raw Front Left angle",
    //   (Units.radiansToDegrees(frontLeft.getRawEncoderValue()))
    // );
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
    System.out.println("Swerve STOP done.");
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    );
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
