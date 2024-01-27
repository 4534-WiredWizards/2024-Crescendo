// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAprilTag extends Command{
  /** Creates a new DriveAprilTag. */
  Limelight limelight;
  SwerveSubsystem swerve; 
  Limelight locLimelight;
  double limelightY;
  double limelightX;
  double limelightTheta;
  boolean inMotion = false;
  private final Supplier<Boolean> cancel;
  Command swerveControllerCommand;
 




  public DriveAprilTag(
    SwerveSubsystem swerve, 
    Limelight llSubsystem,
    Supplier<Boolean> cancel
    ) {

    this.swerve = swerve;
    this.cancel = cancel;
    locLimelight = llSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve, locLimelight);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // limelightX = locLimelight.getXDistance();
    // limelightY = locLimelight.getYDistance();
    // limelightTheta = locLimelight.getThetaDegrees();
    inMotion = true;

    // set network table
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);

    // Trajectory using field oreanted limelight data
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(
    //     locLimelight.botpose.getXDistance(), 
    //     locLimelight.botpose.getYDistance(), 
    //     Rotation2d.fromDegrees(locLimelight.botpose.getThetaDegrees())
    //   ),
    //   List.of(),
    //   new Pose2d(6.4, -3.0, Rotation2d.fromDegrees(0)),
    //   RobotContainer.autoTrajectoryConfig); 
    System.out.println("fbd " + locLimelight.targetpose.getFrontBackDistance());
    System.out.println("degrees " + locLimelight.targetpose.getThetaDegrees());

    // Trajectory using robot oriented limelight data
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //Starting from 0 for target pose because values are relative to single april tag not postion on field
      new Pose2d(
        locLimelight.targetpose.getFrontBackDistance(), 
        // We don't really know why left right dis. should be negitive, but it should be
        -locLimelight.targetpose.getLeftRightDistance(), 
          Rotation2d.fromDegrees(-locLimelight.targetpose.getThetaDegrees())
      ),
      List.of(),
      // Inverts the value of left and right as limelight supplies it relative to the robot, not in the way trajectory needs it
      new Pose2d(
          -1,
          0,
          Rotation2d.fromDegrees(0)
      ),
      RobotContainer.autoTrajectoryConfig
    ); 
      
      

PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
  AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);
SwerveControllerCommand  swerveControllerCommand = new SwerveControllerCommand(
trajectory,
swerve::getPose,
DriveConstants.kDriveKinematics,
xController,
yController,
thetaController,
swerve::setModuleStates,
swerve);
this.swerveControllerCommand = swerveControllerCommand;
// Exectute the commands

// 1. Reset the odometry to the starting pose of the trajectory.
swerve.resetOdometry(trajectory.getInitialPose());
// 2. Run the command!
this.swerveControllerCommand.until(() -> cancel.get()).schedule();




// 3. Wait for the command to finish then stop modules.


    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
