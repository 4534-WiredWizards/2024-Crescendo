// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommandConstants;
import frc.robot.GeneralTrajectories;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.DoNothing;
import frc.robot.commands.PIDMoveArm;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.drivetrain.FollowTrajectory;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{
    TwoNoteMiddle,
    ProLoaded,
    LeaveZone,
    DoNothing
  }
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter shooter;
  private final Arm arm;
  private final ArmProfiledPID ArmProfiledPID;
  private final Intake intake;
  private final Limelight limelight;
  private SendableChooser<AutoMode> autoChooser;
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
    autoChooser.addOption("Two Note Middle", AutoMode.TwoNoteMiddle);
    autoChooser.addOption("Shoot Pre Loaded", AutoMode.ProLoaded);
    autoChooser.addOption("Do Nothing", AutoMode.DoNothing);
    autoChooser.addOption("Leave Zone", AutoMode.LeaveZone);
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DoNothing);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void fetchTrajectories(String AllianceColor, String Trajectory){
    // AutoTrajectories.fetchTrajectories();
  }

  public Command getAuto(){
    AutoMode selectedAutoMode = (AutoMode) (autoChooser.getSelected());

    System.out.println("Running getAuto");
    switch (selectedAutoMode) {
      default:


      // NEW STUFF
      case TwoNoteMiddle:
        System.out.println("Starting Middle Red Note"); 
        limelight.resetLimelightBotPose(); //Resets the swerve odometry pose based on whatever april tag is in view
        System.out.println("After Odom Reset"); 
        String allianceColor = RobotContainer.getAllianceColor();
        System.out.println("AUTO Alliance Color: "+allianceColor);
        

        Command blueAuto = new SequentialCommandGroup(
            new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
              new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)).withTimeout(3),
              new SequentialCommandGroup(
                new DoNothing().withTimeout(1.2), //Wait a for robot to drive back to shooting position
                new RunShooter(shooter, intake, () -> 1.0, false,true, true)
              )            
            ),
            new ParallelCommandGroup(
              new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerNote, true),
              new RunIntake(intake, true, .7, true)
            ),
            new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
              new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
              new SequentialCommandGroup(
                new DoNothing().withTimeout(1), //Wait a for robot to drive back to shooting position
                new RunShooter(shooter, intake, () -> 1.0, false,true, true)
              )
            )
            // new PIDMoveArm(arm, ArmProfiledPID, 0.0)
        );
        Command redAuto = new SequentialCommandGroup(
          new ParallelCommandGroup(
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerShoot, true),
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)).withTimeout(2),
            new RunShooter(shooter, intake, () -> 1.0, false,true, true)
          ),
          new ParallelCommandGroup(
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.intake)),
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerNote, true),
            new RunIntake(intake, true, .7, true)
          ),
          new ParallelCommandGroup(
            new FollowTrajectory(swerveSubsystem, AutoTrajectories.redSpeakerShoot, true),
            new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
            new SequentialCommandGroup(
              new DoNothing().withTimeout(.8), //Wait a for robot to drive back to shooting position
              new RunShooter(shooter, intake, () -> 1.0, false,true, true)
            )
          )
        );
        if (allianceColor == "Blue"){
          autoRoutine = blueAuto;
        } else if (allianceColor == "Red"){
          autoRoutine = redAuto;
        } else {
          autoRoutine = new SequentialCommandGroup();
        }
      break;

      case ProLoaded:
        autoRoutine = new ParallelCommandGroup(
              new FollowTrajectory(swerveSubsystem, AutoTrajectories.blueSpeakerShoot, true),
              new PIDMoveArm(arm,ArmProfiledPID, Units.degreesToRadians(CommandConstants.Arm.closeSpeaker)),
              new SequentialCommandGroup(
                new DoNothing().withTimeout(1), //Wait a for robot to drive back to shooting position
                new RunShooter(shooter, intake, () -> 1.0, false,true, true)
              )
        );
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

