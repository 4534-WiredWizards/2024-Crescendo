// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax topMotor = new CANSparkMax(Constants.SubsystemConstants.ShooterTopCANid,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax bottomMotor = new CANSparkMax(Constants.SubsystemConstants.ShooterBottomCANid,CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topMotor.getEncoder();

  private final SparkPIDController shooterController;
  
  /** Creates a new Shooter. */
  public Shooter() {
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    // Testing for climb
    // topMotor.setIdleMode(IdleMode.kBrake);
    // bottomMotor.setIdleMode(IdleMode.kBrake);
    // bottomMotor.follow(topMotor,true);

    // New PID Controller
    shooterController = topMotor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){
    topMotor.set(speed);
  }

  public void velocityPID(double velocity){
    shooterController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  public double getSpeed(){
    return topEncoder.getVelocity();
  }

}
