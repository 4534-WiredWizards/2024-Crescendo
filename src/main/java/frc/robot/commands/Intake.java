// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMotor;


public class Intake extends Command {
  /** Creates a new IntakeMotor. */
  private final IntakeMotor motors;
  private final DoubleSupplier mspeed;

  boolean m_fwdDir;
  double speedScale = 0.6;
  boolean init_state = true;

  public Intake(
    IntakeMotor subsystem, 
    boolean forwardDirection, 
    DoubleSupplier speed
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
    motors = subsystem;
    m_fwdDir = forwardDirection;
    addRequirements(motors);
    mspeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motors.setBrakeMode();
    init_state = motors.getForwardLimitSwitch();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Front Limit:", motors.getForwardLimitSwitch());
    // SmartDashboard.putBoolean("Back Limit:", motors.getReverseLimitSwitch());
    if (init_state || !motors.getForwardLimitSwitch() || motors.getIsShooter()){
      if(m_fwdDir) {
          if(Math.abs(mspeed.getAsDouble()) < .1){
          motors.move(0);
          }
          else{
            motors.move(Math.pow(mspeed.getAsDouble(), 3));
          }
      } 
      else {
          motors.move(Math.pow(-1*mspeed.getAsDouble(), 3));
      }
    }
  }                                                                                                                                                                                                                                                                                                   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motors.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
