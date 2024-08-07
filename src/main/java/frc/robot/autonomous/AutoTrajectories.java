package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.RobotContainer;
import java.util.List;

public class AutoTrajectories {

  /**
   * Defines the trajectories to be run through auto
   */

  // NEW TRAJECTORIES Using new auto constants

  public static Trajectory red = TrajectoryGenerator.generateTrajectory(
    new Pose2d(5.540121, -2.2, new Rotation2d(Math.toRadians(180))),
    List.of(),
    new Pose2d(
      0, //X
      -2.2, //Y
      new Rotation2d(Math.toRadians(0))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueTaxi = TrajectoryGenerator.generateTrajectory(
    new Pose2d(-5.540121, -2.2, new Rotation2d(Math.toRadians(180))),
    List.of(),
    new Pose2d(
      0, //X
      -2.2, //Y
      new Rotation2d(Math.toRadians(0))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Red Stage Note
  public static Trajectory redStageNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] + 0.1),
      (TrajectoryConstants.red.stageNote[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2] + 28))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0]), //X
      (TrajectoryConstants.red.stageNote[1]), //Y
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2] + 32))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // -------------------- RED AUTO --------------------

  // Red Speaker Note
  public static Trajectory redSpeakerNote = TrajectoryGenerator.generateTrajectory(
    // Red Side is positive, so HIGHER numbers are closer to the drive station wall and
    // LOWER numbers are away from the dlrive station wall
    // Array value 0 is the x value, 1 is the y value
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] + 0.1),
      (TrajectoryConstants.red.speakerNote[1] - 0.01),
      new Rotation2d(TrajectoryConstants.red.speakerNote[2])
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] - 0.4),
      (TrajectoryConstants.red.speakerNote[1] + 0.1),
      new Rotation2d(TrajectoryConstants.red.speakerNote[2])
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Red Amp Note
  public static Trajectory redAmpNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] + .6), //Halfway between 1st and 2nd note
      (TrajectoryConstants.red.ampNote[1] - .2), //Move out towards the subwoofer to allow backing into the 2nd note
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2] - 24))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] - 0.2),
      (TrajectoryConstants.red.ampNote[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2] - 24))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Position to shoot note when when we are on subwoofer
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

  // Four Note AUTO's - Copy of blueStageNoteFour, blueSpeaker
  public static Trajectory redStageNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.speakerShoot[0]), //X
      (TrajectoryConstants.red.speakerShoot[1]), //Y
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2] + 28))
    ),
    List.of(
      new Translation2d(
        (TrajectoryConstants.red.speakerShoot[0] - .9),
        (TrajectoryConstants.red.speakerShoot[1] - .9)
      )
    ),
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0]), //X
      (TrajectoryConstants.red.stageNote[1] - .1), //Y
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.stageNote[2] + 32))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory redSpeakerNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.stageNote[0] - 0.4),
      (TrajectoryConstants.red.stageNote[1] - .1),
      new Rotation2d(TrajectoryConstants.red.stageNote[2])
    ),
    List.of(
      // Add middle point to create arc from 1st to 2nd note and allow backing into the 2nd note
      // 1st note
      new Translation2d(
        (TrajectoryConstants.red.speakerNote[0] + .6), //Halfway between 1st and 2nd note
        (TrajectoryConstants.red.speakerNote[1] - .1) //Move out towards the subwoofer to allow backing into the 2nd note
      )
    ),
    new Pose2d(
      (TrajectoryConstants.red.speakerNote[0] - 0.3),
      (TrajectoryConstants.red.speakerNote[1] + .2),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.speakerNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory redAmpNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] + .6), //Halfway between 1st and 2nd note
      (TrajectoryConstants.red.ampNote[1] - .2), //Move out towards the subwoofer to allow backing into the 2nd note
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2] - 24))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.red.ampNote[0] - 0.2),
      (TrajectoryConstants.red.ampNote[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.red.ampNote[2] - 24))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // ------------------- BLUE AUTO ----------------------------------

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
      (TrajectoryConstants.blue.stageNote[1] + 0.1),
      new Rotation2d(Math.toRadians(-28))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0]),
      (TrajectoryConstants.blue.stageNote[1]),
      new Rotation2d(Math.toRadians(-32))
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
      (TrajectoryConstants.blue.speakerNote[0] + 0.5),
      (TrajectoryConstants.blue.speakerNote[1] + 0.1),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  // Blue Amp Note
  public static Trajectory blueAmpNote = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] - .6),
      (TrajectoryConstants.blue.ampNote[1] - .2),
      new Rotation2d(Math.toRadians(24))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] + 0.2),
      (TrajectoryConstants.blue.ampNote[1] + 0.1),
      new Rotation2d(Math.toRadians(24))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueStageNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] + .15),
      (TrajectoryConstants.blue.stageNote[1] - .1),
      new Rotation2d(Math.toRadians(-32))
    ),
    List.of(),
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] - 1.1),
      (TrajectoryConstants.blue.stageNote[1] + .3),
      new Rotation2d(Math.toRadians(-32))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueSpeakerNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.stageNote[0] + 0.4),
      (TrajectoryConstants.blue.stageNote[1] - .1),
      new Rotation2d(TrajectoryConstants.blue.stageNote[2])
    ),
    List.of(
      // Add middle point to create arc from 1st to 2nd note and allow backing into the 2nd note
      // 1st note
      new Translation2d(
        (TrajectoryConstants.blue.speakerNote[0] - .6), //Halfway between 1st and 2nd note
        (TrajectoryConstants.blue.speakerNote[1] - .1) //Move out towards the subwoofer to allow backing into the 2nd note
      )
    ),
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] + 0.2),
      (TrajectoryConstants.blue.speakerNote[1] + .2),
      new Rotation2d(Math.toRadians(TrajectoryConstants.blue.speakerNote[2]))
    ),
    RobotContainer.autoTrajectoryConfig
  );

  public static Trajectory blueAmpNoteFour = TrajectoryGenerator.generateTrajectory(
    new Pose2d(
      (TrajectoryConstants.blue.speakerNote[0] + 0.4),
      (TrajectoryConstants.blue.speakerNote[1] - .1),
      new Rotation2d(TrajectoryConstants.angleToSubwoofer + 180)
    ),
    List.of(
      // Add middle point to create arc from 1st to 2nd note and allow backing into the 2nd note
      // 1st note
      new Translation2d(
        (TrajectoryConstants.blue.ampNote[0] - .6), //Halfway between 1st and 2nd note
        (TrajectoryConstants.blue.ampNote[1] - .2) //Move out towards the subwoofer to allow backing into the 2nd note
      )
    ),
    new Pose2d(
      (TrajectoryConstants.blue.ampNote[0] + 0.2),
      (TrajectoryConstants.blue.ampNote[1] + 0.1),
      new Rotation2d(Math.toRadians(24))
    ),
    RobotContainer.autoTrajectoryConfig
  );
}
