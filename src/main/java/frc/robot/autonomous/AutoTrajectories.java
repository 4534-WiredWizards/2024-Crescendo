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
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(-2)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(-90), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-60)),
                new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))

                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(10))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-50), Units.inchesToMeters(-2), new Rotation2d(Math.toRadians(0))),
            config
    );


   


   


    


}