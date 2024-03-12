// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.aruco.Aruco;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.GeneralTrajectories;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Limelight.Botpose;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{
    TwoNoteMiddle,
    PreLoaded,
    LeaveZone,
    ShootAndLeave,
    Side2Note,
    DoNothing
  }
  public enum AllianceColor{
    Red,
    Blue
  }
  public enum SubWooferSide {
    RedAmp,
    RedStage,
    BlueAmp,
    BlueStage,
    Unknown
  }
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter shooter;
  private final Arm arm;
  private final ArmProfiledPID ArmProfiledPID;
  private final Intake intake;
  private final Limelight limelight;
  private SendableChooser<AutoMode> autoChooser;
  private SendableChooser<AllianceColor> allianceColorChooser;
  private Command autoRoutine;
  private ParallelCommandGroup shootNoteWhenOnSub; 
  
  /** Creates a new AutoChooser. */
  public AutoChooser(
    SwerveSubsystem SwerveSubsystem,
    Shooter Shooter,
    Intake intake,
    Arm Arm,
    ArmProfiledPID ArmProfiledPID,
    Limelight Limelight
  ) {
    this.swerveSubsystem = SwerveSubsystem;
    this.shooter = Shooter;
    this.arm = Arm;
    this.ArmProfiledPID = ArmProfiledPID;
    this.intake = intake;
    this.limelight = Limelight;
    autoChooser = new SendableChooser<AutoMode>();
    autoChooser.addOption("Two Note Middle(TAG)", AutoMode.TwoNoteMiddle);
    autoChooser.addOption("Pre Loaded", AutoMode.PreLoaded);
    autoChooser.addOption("Side 2 Note", AutoMode.Side2Note);
    autoChooser.addOption("Do Nothing", AutoMode.DoNothing);
    // autoChooser.addOption("Leave Zone(NO TAG)", AutoMode.LeaveZone);
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DoNothing);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Drop Down Menu For Alliance Selection
    allianceColorChooser = new SendableChooser<AllianceColor>();
    allianceColorChooser.addOption("Red", AllianceColor.Red);
    allianceColorChooser.addOption("Blue", AllianceColor.Blue);
    SmartDashboard.putData("Alliance Color", allianceColorChooser);


    // Constants for auto
    this.shootNoteWhenOnSub = new ParallelCommandGroup(
      new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
      new SequentialCommandGroup(
        new DoNothing().withTimeout(1.2), //Wait a for robot to drive back to shooting position
        new RunShooter(shooter, intake, () -> 1.0, false,true, true)
      )
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void fetchTrajectories(String AllianceColor, String Trajectory){
    // AutoTrajectories.fetchTrajectories();
  }
  
  public SubWooferSide getSubWooferSide(AllianceColor allianceColor, Double botYPose){
    // If on the blue alliance
    if (allianceColor == AllianceColor.Blue){
      if (botYPose > TrajectoryConstants.centerOfSubwoofer){
        return SubWooferSide.BlueAmp;
      } else {
        return SubWooferSide.BlueStage;
      }
    } else {
      if (botYPose > TrajectoryConstants.centerOfSubwoofer){
        return SubWooferSide.RedAmp;
      } else {
        return SubWooferSide.RedStage;
      }
    }
  }


  public Command getAuto(){
    AutoMode selectedAutoMode = (AutoMode) (autoChooser.getSelected());
    AllianceColor selectedAllianceColor = (AllianceColor) (allianceColorChooser.getSelected());
    Double botXPose = limelight.botpose.getXDistance();
    Double botYPose = limelight.botpose.getYDistance();
    Double botRotation = limelight.botpose.getThetaDegreesField();
    SubWooferSide subWooferSide = getSubWooferSide(selectedAllianceColor, botYPose);

    
    // System.out.println("Running getAuto");
    switch (selectedAutoMode) {
      default:


      // NEW STUFF
      case TwoNoteMiddle:
        Boolean twoNoteResetOdom = limelight.resetLimelightBotPose(botXPose,botYPose,botRotation);
        System.out.println("Starting Middle Note"); 
      
        //Resets the swerve odometry pose based on whatever april tag is in view
        // System.out.println("After Odom Reset"); 
        // String allianceColor = RobotContainer.getAllianceColor();
        AllianceColor fetchColor = selectedAllianceColor;
        Command blueAuto = new SequentialCommandGroup(
            new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
              shootNoteWhenOnSub     
            ),
            new ParallelCommandGroup(
              new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerNote, true),
              new RunIntake(intake, true, .7, true)
            ),
            new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
             shootNoteWhenOnSub
            )
            // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
        );
        Command redAuto = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerShoot, true),
            shootNoteWhenOnSub        
          ),
          new ParallelCommandGroup(
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerNote, true),
            new RunIntake(intake, true, .7, true)
          ),
          new ParallelCommandGroup(
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerShoot, true),
            shootNoteWhenOnSub
          )
        );
          // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
        if (fetchColor == AllianceColor.Blue && twoNoteResetOdom){
          System.out.println("Blue Two Note Auto");
          autoRoutine = blueAuto;
        } else if (fetchColor == AllianceColor.Red && twoNoteResetOdom){
          System.out.println("Red Two Note Auto");
          autoRoutine = redAuto;
        } else {
          autoRoutine = shootNoteWhenOnSub;
        }
      break;

      case PreLoaded:
        autoRoutine = shootNoteWhenOnSub;
      break;

      
      // case ShootAndLeave:
      // autoRoutine = new SequentialCommandGroup(
      //   new ParallelCommandGroup(
      //     new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
      //     new SequentialCommandGroup(
      //       new DoNothing().withTimeout(2), //Wait a for robot to drive back to shooting position
      //       new RunShooter(shooter, intake, () -> 1.0, false,true, true)
      //     )
      //   )
      //   // new GeneralTrajectories().Back(swerveSubsystem)
      //   // TODO: Add logic
      // );
      // break;

      case Side2Note:
        System.out.println("Starting Middle Note"); 
        Boolean Side2NoteResetOdom = limelight.resetLimelightBotPose(botXPose,botYPose,botRotation);
        Command redAmp = new SequentialCommandGroup(
          shootNoteWhenOnSub,    
          new ParallelCommandGroup(
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redAmpNote, true),
            new RunIntake(intake, true, .7, true)
          ),
          new ParallelCommandGroup(
            // new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
            // TODO: Move back to where you started
           shootNoteWhenOnSub
          )
          // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
      );


        Command redStage = new SequentialCommandGroup(null);
        Command blueAmp = new SequentialCommandGroup(null);
        Command blueStage = new SequentialCommandGroup(null);
        if (subWooferSide == SubWooferSide.RedAmp && Side2NoteResetOdom) {
          autoRoutine = redAmp;
        } else if (subWooferSide == SubWooferSide.RedStage && Side2NoteResetOdom) {
          autoRoutine = redStage;
        } else if (subWooferSide == SubWooferSide.BlueAmp && Side2NoteResetOdom) {
          autoRoutine = blueAmp;
        } else if (subWooferSide == SubWooferSide.BlueStage && Side2NoteResetOdom) {
          autoRoutine = blueStage;
        } else {
          autoRoutine = shootNoteWhenOnSub;
          // TODO: Add logic for static exit auto
        }
      break;

      
      
      case LeaveZone:
        autoRoutine = new GeneralTrajectories().Back(swerveSubsystem);
      break;



      // WOKRING
      // case BlueAutoTest: //Working
      //   System.out.println("Starting Middle Blu Note"); 
      //   limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
      //   System.out.println("After Odom Reset"); 
      //   autoRoutine = new SequentialCommandGroup(
      //       // new  InstantCommand(() -> System.out.println("Moving to note, "+TrajectoryConstants.blue.speakerNote[0]+", "+TrajectoryConstants.blue.speakerNote[1])),
      //       new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerNote, true)
      //   );
      // break;
      // case RedAutoTest: //Working
      //   System.out.println("Starting Middle Red Note"); 
      //   limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
      //   System.out.println("After Odom Reset"); 
      //   autoRoutine = new SequentialCommandGroup(
      //       // new  InstantCommand(() -> System.out.println("Moving to note, "+TrajectoryConstants.red.speakerNote[0]+", "+TrajectoryConstants.blue.speakerNote[1])),
      //       new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerNote, true)
      //   );
      // break;




      // OLD STUFF

      // case BlueMiddleNote:
      //   System.out.println("Starting Middle Blue Note"); 
      //   limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
      //   System.out.println("After Odom Reset"); 
      //   autoRoutine = new SequentialCommandGroup(
       
      //       new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueMiddleNote, false),
      //       new DoNothing().withTimeout(1),
      //       new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueStageNote, true)

           
      //   );
      // break;



      // case MultiSpeaker:
      // //Specific setpoints not finished, remove this line when tested and finished
      // autoRoutine = new SequentialCommandGroup(
        
      // );
      // break;

      // case AmpScore:
      //  //Specific setpoints not finished, remove this line when tested and finished
      // autoRoutine = new SequentialCommandGroup(
      //   // new InstantCommand(() -> limelight.resetLimelightPose()),
      //   new GeneralTrajectories().toTag(swerveSubsystem),
      //   new PIDMoveArm(arm, ArmProfiledPID, 0.0)
      //   // new RunShooter(shooter, intake, () -> 1.0, false,true)
      // );
      // break;

      // case Driveout:
      // autoRoutine = new GeneralTrajectories().Back(swerveSubsystem);
      // break;

     
     
      case DoNothing:
        autoRoutine = new DoNothing();
  }
  return autoRoutine;
}
}

