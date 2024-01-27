package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class TrajectoryTest {

    
    public Command getTrajectory(SwerveSubsystem swerve, Limelight limelight){
        System.out.println("test");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //Starting from 0 for target pose because values are relative to single april tag not postion on field
      new Pose2d(
        -1,
        // We don't really know why left right dis. should be negitive, but it should be
        .1, 
          Rotation2d.fromDegrees(0)),
      List.of(),
      // Inverts the value of left and right as limelight supplies it relative to the robot, not in the way trajectory needs it
      new Pose2d(
          -1,
          0,
          Rotation2d.fromDegrees(1)
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
return swerveControllerCommand;
}
}

