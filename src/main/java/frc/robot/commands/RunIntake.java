// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RunIntake extends Command {
  /** Creates a new IntakeMotor. */

  private final DoubleSupplier mspeed;

  boolean m_fwdDir;
  double speedScale = 0.6;
  boolean init_state = true;
  Intake intake;

  public RunIntake(
    Intake intake,
    boolean forwardDirection, 
    DoubleSupplier speed
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    m_fwdDir = forwardDirection;
    addRequirements(intake);
    mspeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_state = intake.getForwardLimitSwitch();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (init_state || !intake.getForwardLimitSwitch()){
      if(m_fwdDir) {
          if(Math.abs(mspeed.getAsDouble()) < .1){
          intake.move(0);
          }
          else{
            intake.move(Math.pow(mspeed.getAsDouble(), 3));
          }
      } 
      else {
          intake.move(Math.pow(-1*mspeed.getAsDouble(), 3));
      }
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
    return false;
  }

}
