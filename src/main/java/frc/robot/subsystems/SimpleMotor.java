// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SimpleMotor  extends SubsystemBase {
    /** Creates a new SimpleMotor. */
  private CANSparkMax motor1;
 
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  
  public RelativeEncoder m_encoder;

  private double calculatedDistance;
  private int CANSparkMaxID;
  private String MotorMoveType;


  public SimpleMotor(
    int motorid,
    String motorkind,
    boolean isBrushless
    // Anybody that sees this needs to use these ->
    // boolean hasLimitSwitch
    // boolean isNormallyOpen
  ) {
    CANSparkMaxID=motorid;
    MotorMoveType=motorkind;
    if (isBrushless) {
      motor1 = new CANSparkMax(CANSparkMaxID, CANSparkLowLevel.MotorType.kBrushless);
    }
    else {
      motor1 = new CANSparkMax(CANSparkMaxID, CANSparkLowLevel.MotorType.kBrushed);
    }

  //we are inverting direction to match the motor movement
    motor1.setInverted(true);
    // m_forwardLimit = motor1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_reverseLimit = motor1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_forwardLimit.enableLimitSwitch(true);
    // m_reverseLimit.enableLimitSwitch(true);
    m_encoder = motor1.getEncoder();
    m_encoder.setPositionConversionFactor(28.125);
  



  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    // SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
    // SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    // SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    // SmartDashboard.putNumber("Camera Distance", getDistance());
    // SmartDashboard.putNumber("Calcualted Tag Distance", calculatedDistance());
  
    //motor1.set(0.2);
    // This method will be called once per scheduler run
  }
  public void move(double speed) {
    // System.out.println(speed);
    motor1.set(speed);
  }

  
  // public boolean getForwardLimitSwitch(){
  //   return m_forwardLimit.isPressed();
  // }

  // public boolean getReverseLimitSwitch(){
  //   return m_reverseLimit.isPressed();
  // }

  public void setBrakeMode(){
    motor1.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    motor1.setIdleMode(IdleMode.kCoast);
  }

  //Make function to reset encoder value
  public void resetMotorEncoder(){
    m_encoder.setPosition(0);
  }

  public double getEncoderPosition(){
    return m_encoder.getPosition();
  }

  public double getSpeed(){
    return m_encoder.getVelocity();
  }
}
