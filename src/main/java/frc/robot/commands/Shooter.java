// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleMotor;
import frc.robot.subsystems.IntakeMotor;

public class Shooter extends Command {
  /** Creates a new MoveMotor. */
  private final SimpleMotor motor2;
  private final IntakeMotor intake;
  private final DoubleSupplier mspeed;
  boolean m_fwdDir;
  boolean bPressed2;
  double speedScale = 0.6;


  public Shooter(
    SimpleMotor subsystem,
    IntakeMotor localIntake,
    boolean forwardDirection, 
    DoubleSupplier speed
    ) {
    // Use addRequirements() here to declare subsystem dependencies.
    motor2 = subsystem;
    intake = localIntake;
    m_fwdDir = forwardDirection;
    addRequirements(motor2);
    mspeed = speed;
    SmartDashboard.putNumber("Shooter Speed", speedScale);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor2.setCoastMode();
    // Imports motor1 from SimpleMotor.java and gets encoder to
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Front Limit:", motor2.getForwardLimitSwitch());
    // SmartDashboard.putBoolean("Back Limit:", motor2.getReverseLimitSwitch());
    intake.isShooting(true);
    speedScale = SmartDashboard.getNumber("Shooter Speed", speedScale);
      System.out.println("speed =" + (.5 * (Math.pow(mspeed.getAsDouble(), 3))));
    if(m_fwdDir) {
      
   
        if(Math.abs(mspeed.getAsDouble()) < .1){
        motor2.move(0);
        }
        else{
          motor2.move(speedScale * (Math.pow(mspeed.getAsDouble(), 3)));
        }
      
      
      // motor2.move(.2);
    } 
    else {
        motor2.move(-speedScale * (Math.pow(mspeed.getAsDouble(), 3)));
    }
  }                                                                                                                                                                                                                                                                                                   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor2.move(0);
    intake.isShooting(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

