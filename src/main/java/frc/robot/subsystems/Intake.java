// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkLimitSwitch forwardLimitSwitch;
  private final CANSparkMax intakeMotor = new CANSparkMax(50, CANSparkLowLevel.MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    forwardLimitSwitch = intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forwardLimitSwitch.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){
    intakeMotor.set(speed);
  }
  public boolean getForwardLimitSwitch(){
   return forwardLimitSwitch.isPressed();
  }
}
