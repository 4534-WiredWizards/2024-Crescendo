// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RunIntake extends Command {
  /** Creates a new IntakeMotor. */

  boolean m_fwdDir;
  double speedScale = 0.6;
  boolean init_state = true;
  Intake intake;
  boolean autostop;
  double mspeed;

  public RunIntake(
    Intake intake,
    boolean forwardDirection, 
    Double speed,
    boolean autostop
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    m_fwdDir = forwardDirection;
    mspeed = speed;
    this.autostop = autostop;
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_state = intake.getIntakeStatus();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      if(m_fwdDir) {
          if(Math.abs(mspeed) < .1){
          intake.move(0);
          }
          else{
            intake.move(Math.pow(mspeed, 3));
          }
      } 
      else {
          intake.move(Math.pow(-1*mspeed, 3));
      }
  }                                                                                                                                                                                                                                                                                                   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.move(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(autostop){
      return !(init_state || !intake.getIntakeStatus());
    }
    else{
      return false;
    }
  }

}
