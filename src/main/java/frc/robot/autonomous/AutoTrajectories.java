package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.RobotContainer;
import java.util.List;

public class AutoTrajectories {

  /**
   * Defines the trajectories to be run through auto
   */

  // configures the maximum velocity and accel for the trajectories

  // public static Trajectory redMiddleNote =
  // TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(6, 1.5478, new Rotation2d(Math.toRadians(0))),
  //     List.of(
  //         // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
  //     ),
  //     new Pose2d(5.616321, 1.4478, new Rotation2d(Math.toRadians(0))),
  //     RobotContainer.autoTrajectoryConfig
  // );

  // public static Trajectory blueMiddleNote =
  //     TrajectoryGenerator.generateTrajectory(
  //         new Pose2d(-6.540121, 2.44, new Rotation2d(0)), //Move to center point before note
  //         List.of(
  //             // new Translation2d(1, 1)
  //         ),
  //         new Pose2d(-5.540121, 1.44, new Rotation2d(0)), //Mode to blue middle note
  //         RobotContainer.autoTrajectoryConfig
  // );

  // public static Trajectory blueStageNote =
  //     TrajectoryGenerator.generateTrajectory(
  //         new Pose2d(-6.540121, -.1, new Rotation2d(0)), //Move to center point before note
  //         List.of(
  //             // new Translation2d(1, 1)
  //         ),
  //         new Pose2d(-5.540121, 0.1, new Rotation2d(0)), //Mode to blue middle note
  //         RobotContainer.autoTrajectoryConfig
  // );

  // NEW TRAJECTORIES Using new auto constants

  // Red Stage Note
  public static Trajectory redStageNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] + 0.1),
      (TrajectoryConstants.red.stageNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] - 0.0),
      (TrajectoryConstants.red.stageNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory redStageFourNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] + 0.1),
      (TrajectoryConstants.red.stageNote[1] + 0.5),
      new Rotation2d(Math.toRadians(-90))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] - 0.0),
      (TrajectoryConstants.red.stageNote[1] + 0.0),
      new Rotation2d(Math.toRadians(-90))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Red Speaker Note
  public static Trajectory redSpeakerNote = TrajectoryGenerator.generateTrajectory(
    // Red Side is positive, so HIGHER numbers are closer to the drive station wall and
    // LOWER numbers are away from the dlrive station wall
    // Array value 0 is the x value, 1 is the y value
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] + 0.1),
      (TrajectoryConstants.red.speakerNote[1] - 0.01),
      new Rotation2d(Math.toRadians(-90))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] - 0.4),
      (TrajectoryConstants.red.speakerNote[1] + 0.1),
      new Rotation2d(Math.toRadians(-90))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory redSpeakerFourNote = TrajectoryGenerator.generateTrajectory(
    // Red Side is positive, so HIGHER numbers are closer to the drive station wall and
    // LOWER numbers are away from the dlrive station wall
    // Array value 0 is the x value, 1 is the y value
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] + 0.1),
      (TrajectoryConstants.red.speakerNote[1] - 0.01),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.speakerNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] - 0.4),
      (TrajectoryConstants.red.speakerNote[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.speakerNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Red Amp Note
  public static Trajectory redAmpNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] + 0.1),
      (TrajectoryConstants.red.ampNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] - 0.0),
      (TrajectoryConstants.red.ampNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory redAmpFourNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      TrajectoryConstants.red.ampNote[0] + 1,
      (TrajectoryConstants.red.ampNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2]))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0]),
      (TrajectoryConstants.red.ampNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Red Speaker Shoot
  public static Trajectory redSpeakerShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.speakerShoot[0] + 0.3),
      (TrajectoryConstants.red.speakerShoot[1] + 0.01),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.speakerShoot[2]))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.speakerShoot[0] + 0.4),
      (TrajectoryConstants.red.speakerShoot[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.speakerShoot[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Blue Speaker Shoot
  // Location Just in front of the blue speaker note that preloaded note can be scored from
  public static Trajectory blueSpeakerShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.speakerShoot[0] - .3),
      (TrajectoryConstants.blue.speakerShoot[1] + 0.01),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerShoot[2]))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.speakerShoot[0] - .4),
      (TrajectoryConstants.blue.speakerShoot[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerShoot[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Blue Stage Note
  public static Trajectory blueStageNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] - 0.1),
      (TrajectoryConstants.blue.stageNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.stageNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] + 0.0),
      (TrajectoryConstants.blue.stageNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.stageNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueStageNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] - 0.1), //X
      (TrajectoryConstants.blue.stageNote[1] + 1), //Y
      new Rotation2d(TrajectoryConstants.angleToSubwoofer)
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] + 0.4),
      (TrajectoryConstants.blue.stageNote[1] - .1),
      new Rotation2d(TrajectoryConstants.angleToSubwoofer)
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Blue Speaker Note
  public static Trajectory blueSpeakerNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] - 0.1),
      (TrajectoryConstants.blue.speakerNote[1] + 0.01),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerNote[2]))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] + 0.4),
      (TrajectoryConstants.blue.speakerNote[1] - 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueSpeakerFourNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] - 0.1),
      (TrajectoryConstants.blue.speakerNote[1] + 0.01),
      new Rotation2d(Math.toRadians(-90))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] + 0.4),
      (TrajectoryConstants.blue.speakerNote[1] - 0.1),
      new Rotation2d(Math.toRadians(-90))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Blue Amp Note
  public static Trajectory blueAmpNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] - 0.1),
      (TrajectoryConstants.blue.ampNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.ampNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] + 0.0),
      (TrajectoryConstants.blue.ampNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.ampNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueAmpFourNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] + 1),
      (TrajectoryConstants.blue.ampNote[1] + 0.5),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.ampNote[2]))
    ),
    List.of(
      // new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
    ),
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] + 0.0),
      (TrajectoryConstants.blue.ampNote[1] + 0.0),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.ampNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );
}
