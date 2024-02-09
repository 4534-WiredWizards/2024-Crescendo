// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimbMotor = new CANSparkMax(Constants.SubsystemConstants.ClimbLeftCANid,CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightClimbMotor = new CANSparkMax(Constants.SubsystemConstants.ClimbRightCANid,CANSparkLowLevel.MotorType.kBrushless);
  private final SparkLimitSwitch forwardLimitSwitch;
  public final RelativeEncoder climbEncoder;

  /** Creates a new Climb. */
  public Climb() {
    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setInverted(true);
    rightClimbMotor.follow(leftClimbMotor);
    forwardLimitSwitch = leftClimbMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    forwardLimitSwitch.enableLimitSwitch(true);
    climbEncoder = leftClimbMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){
    // Positive is winding?
    leftClimbMotor.set(speed);
  }

  public double getPosition() {
    // return encoder value (in rotations)
    return climbEncoder.getPosition();
  }

  public void EncoderReset() {
    // return encoder value (in rotations)
    climbEncoder.setPosition(0);
  }


  public boolean getClimbStatus(){
    return forwardLimitSwitch.isPressed();
  }
}