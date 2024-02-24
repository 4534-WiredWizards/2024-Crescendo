package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AprilTagPositions;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class GeneralTrajectories {
    SwerveSubsystem swerveSubsystem;
    public Command Back(SwerveSubsystem swerveSubsystem){
        //Only commenting first trajectory since it repeats
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start and end are both waypoints, and the robot will go from the starting pose to both of them
            //Values are relative to where we reset odometry to, mostly the april tag will be 0,0
            //For some reason, when the degrees dont change the robot just stutter steps so add at least a 1 degree change

            //This one moves back 1.5 meters from a point 1 meter away from the tag(Meant to be used with toTag)
                new Pose2d(-1, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(
                    -2.5, 0
                , Rotation2d.fromDegrees(1)),
                RobotContainer.autoTrajectoryConfig);      


        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        return swerveControllerCommand;
    }

    public Command toTag(SwerveSubsystem swerve){
        //Moves to 1 meter in front of the tag
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(
        -1,
        .1, 
          Rotation2d.fromDegrees(0)),
      List.of(),
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

    public Command toStraightBackNote(SwerveSubsystem swerve){
        //Moves back from the april tag to the note(Meant to be used with toTag)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
              -1,
              0, 
                Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(
                -6,
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

    public Command toLeftBackNote(SwerveSubsystem swerve){
        //Moves back and right from the april tag to the note(Meant to be used with toTag)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
              -1,
              0, 
                Rotation2d.fromDegrees(0)),
            List.of(
                new Translation2d(-1, 1),
                new Translation2d(-6, 1)
            ),
            new Pose2d(
                -6,
                1,
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

    public Command PointToSpeaker(SwerveSubsystem swerve, Supplier<Pose2d> position) {
        Pose2d tempPose = position.get();
        double botX = tempPose.getX();
        double botY = tempPose.getY();
        Rotation2d botRot = tempPose.getRotation();
        double angle = Math.atan(AprilTagPositions.Tag4_x - botX / AprilTagPositions.Tag4_y - botY);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(
              botX,
              botY, 
                Rotation2d.fromDegrees(botRot.getDegrees())),
            List.of(),
            new Pose2d(
                botX,
                botY,
                Rotation2d.fromDegrees(angle)
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
