// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.TreeUI;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax rightmotor;
  private CANSparkMax leftmotor;
  private DutyCycleEncoder throttleEnc;
  private SparkLimitSwitch forwardLimitSwitch;
  private SparkLimitSwitch reverseLimitSwitch;

  /** Creates a new ShooterArm. */
  public Arm() {
    DutyCycleEncoder throttleEnc =  new DutyCycleEncoder(1);
    
    rightmotor = new CANSparkMax(Constants.SubsystemConstants.ArmRIghtCANid, CANSparkLowLevel.MotorType.kBrushless);
    leftmotor = new CANSparkMax(Constants.SubsystemConstants.ArmLeftCANid, CANSparkLowLevel.MotorType.kBrushless);

    rightmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);
    rightmotor.setInverted(true);
    rightmotor.follow(leftmotor);
    
    forwardLimitSwitch = leftmotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = leftmotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forwardLimitSwitch.enableLimitSwitch(true);
    reverseLimitSwitch.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){
    rightmotor.set(speed);
  }

  public double getAbsolutePosition() {
    return throttleEnc.getAbsolutePosition();
  }

  public boolean getArmStatusFw(){
    return forwardLimitSwitch.isPressed();
   }

  public boolean getArmStatusRv(){
    return reverseLimitSwitch.isPressed();
    }
}

