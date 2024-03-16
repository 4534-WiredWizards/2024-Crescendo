// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.GeneralTrajectories;
import frc.robot.commands.DoNothing;
import frc.robot.commands.AutoRoutines.shootNoteWhenOnSub;
import frc.robot.commands.AutoRoutines.centerFourNote.blueFourNoteCenter;
import frc.robot.commands.AutoRoutines.centerFourNote.redFourNoteCenter;
import frc.robot.commands.AutoRoutines.centerTwoNote.blueTwoNoteCenter;
import frc.robot.commands.AutoRoutines.centerTwoNote.redTwoNoteCenter;
import frc.robot.commands.AutoRoutines.sideTwoNote.blueAmpTwoNoteSide;
import frc.robot.commands.AutoRoutines.sideTwoNote.blueStageTwoNoteSide;
import frc.robot.commands.AutoRoutines.sideTwoNote.redAmpTwoNoteSide;
import frc.robot.commands.AutoRoutines.sideTwoNote.redStageTwoNoteSide;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{
    TwoNoteMiddle,
    PreLoaded,
    LeaveZone,
    ShootAndLeave,
    Side2Note,
    FourNoteCenter,
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
    autoChooser.addOption("Mid 2N", AutoMode.TwoNoteMiddle);
    autoChooser.addOption("Pre 1N", AutoMode.PreLoaded);
    autoChooser.addOption("Side 2N", AutoMode.Side2Note);
    autoChooser.addOption("Mid 4N", AutoMode.FourNoteCenter);
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
        
          // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
        if (selectedAllianceColor == AllianceColor.Blue && twoNoteResetOdom){
          System.out.println("Blue Two Note Auto");
          autoRoutine = new blueTwoNoteCenter(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
        } else if (selectedAllianceColor == AllianceColor.Red && twoNoteResetOdom){
          System.out.println("Red Two Note Auto");
          autoRoutine = new redTwoNoteCenter(swerveSubsystem, arm, ArmProfiledPID, intake, shooter);
        } else {
          autoRoutine = new shootNoteWhenOnSub(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
        }
      break;

      case PreLoaded:
        autoRoutine = new shootNoteWhenOnSub(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
      break;


      case Side2Note:
        System.out.println("Starting Middle Note"); 
        Boolean Side2NoteResetOdom = limelight.resetLimelightBotPose(botXPose,botYPose,botRotation);
       

        if (subWooferSide == SubWooferSide.RedAmp && Side2NoteResetOdom) {
          autoRoutine = new redAmpTwoNoteSide(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
        } else if (subWooferSide == SubWooferSide.RedStage && Side2NoteResetOdom) {
          autoRoutine = new redStageTwoNoteSide(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
        } else if (subWooferSide == SubWooferSide.BlueAmp && Side2NoteResetOdom) {
          autoRoutine = new blueAmpTwoNoteSide(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
        } else if (subWooferSide == SubWooferSide.BlueStage && Side2NoteResetOdom) {
          autoRoutine = new blueStageTwoNoteSide(arm, ArmProfiledPID, intake, shooter, swerveSubsystem);
        } else {
          autoRoutine = new shootNoteWhenOnSub(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
          // TODO: Add logic for static exit auto
        }
      break;

      
      
      case LeaveZone:
        autoRoutine = new GeneralTrajectories().Back(swerveSubsystem);
      break;

      case FourNoteCenter:


        Boolean fourNoteResetOdom = limelight.resetLimelightBotPose(botXPose,botYPose,botRotation);
        System.out.println("Starting Four Note"); 
    
     
      
        // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
      if (selectedAllianceColor == AllianceColor.Blue && fourNoteResetOdom){
        System.out.println("Blue Two Note Auto");
        autoRoutine = new blueFourNoteCenter(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
      } else if (selectedAllianceColor == AllianceColor.Red && fourNoteResetOdom){
        System.out.println("Red Two Note Auto");
        autoRoutine = new redFourNoteCenter(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
      } else {
        autoRoutine = new shootNoteWhenOnSub(arm, ArmProfiledPID, intake, swerveSubsystem, shooter);
      }
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

