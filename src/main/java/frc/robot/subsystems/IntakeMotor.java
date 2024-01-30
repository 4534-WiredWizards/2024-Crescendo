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
public class IntakeMotor extends SubsystemBase {

      /** Creates a new IntakeMotor. */
  private CANSparkMax m_intake;
 
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  
  public RelativeEncoder m_encoder;

  private int CANSparkMaxID;
  private String MotorMoveType;

  boolean shooterRunning;
    
  public IntakeMotor(
    int motorid,
    String motorkind,
    boolean isBrushless,
    boolean LimitSwitchUsed
    ) {
    CANSparkMaxID=motorid;
    MotorMoveType=motorkind;
    
    if (isBrushless) {
      m_intake = new CANSparkMax(CANSparkMaxID, CANSparkLowLevel.MotorType.kBrushless);
    }
    else {
      m_intake = new CANSparkMax(CANSparkMaxID, CANSparkLowLevel.MotorType.kBrushed);
    }

  //we are inverting direction to match the motor movement
    m_intake.setInverted(true);
    m_forwardLimit.enableLimitSwitch(false);
    m_forwardLimit = m_intake.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    // m_reverseLimit = m_intake.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_forwardLimit.enableLimitSwitch(true);
    // m_reverseLimit.enableLimitSwitch(true);
    m_encoder = m_intake.getEncoder();
    m_encoder.setPositionConversionFactor(28.125);
  

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Forward Limit Switch", m_forwardLimit.isPressed());
    // SmartDashboard.putBoolean("Intake Reverse Limit Switch", m_reverseLimit.isPressed());
    // SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    // SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    // SmartDashboard.putNumber("Camera Distance", getDistance());
    // SmartDashboard.putNumber("Calcualted Tag Distance", calculatedDistance());
  
    // This method will be called once per scheduler run

  }
  public void move(double speed) {
    // System.out.println(speed);
    m_intake.set(speed);
  }

  
  public boolean getForwardLimitSwitch(){
    return m_forwardLimit.isPressed();
  }

  // public boolean getReverseLimitSwitch(){
  //   return m_reverseLimit.isPressed();
  // }

  public void setBrakeMode(){
    m_intake.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    m_intake.setIdleMode(IdleMode.kCoast);
  };

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

  public void isShooting(boolean value) {
    shooterRunning = value;
  }

  public boolean getIsShooter() {
    return shooterRunning;
  }
}
