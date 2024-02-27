package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class AutoTrajectories {

    /**
     * Defines the trajectories to be run through auto
     */

    // configures the maximum velocity and accel for the trajectories
    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(DriveConstants.kDriveKinematics);

   
    public static Trajectory redMiddleNote =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(6.616321, 1.5478, new Rotation2d(Math.toRadians(0))),
        List.of(
            // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
        ),
        new Pose2d(5.616321, 1.4478, new Rotation2d(Math.toRadians(1))),
        RobotContainer.autoTrajectoryConfig
    );


   


   


    


}