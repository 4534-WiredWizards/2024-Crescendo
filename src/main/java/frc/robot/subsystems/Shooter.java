// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax topMotor = new CANSparkMax(51,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax bottomMotor = new CANSparkMax(52,CANSparkLowLevel.MotorType.kBrushless);
  /** Creates a new Shooter. */
  public Shooter() {
    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.follow(topMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){
    topMotor.set(speed);
  }
}
