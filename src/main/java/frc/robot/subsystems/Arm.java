// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import javax.swing.plaf.TreeUI;

public class Arm extends SubsystemBase {

  private CANSparkMax rightmotor;
  private CANSparkMax leftmotor;
  private DutyCycleEncoder absEncoder;
  private SparkLimitSwitch leftForwardLimitSwitch;
  private SparkLimitSwitch leftReverseLimitSwitch;

  /** Creates a new ShooterArm. */
  public Arm() {
    absEncoder = new DutyCycleEncoder(1);

    rightmotor =
      new CANSparkMax(
        Constants.SubsystemConstants.ArmRIghtCANid,
        CANSparkLowLevel.MotorType.kBrushless
      );
    leftmotor =
      new CANSparkMax(
        Constants.SubsystemConstants.ArmLeftCANid,
        CANSparkLowLevel.MotorType.kBrushless
      );

    rightmotor.restoreFactoryDefaults();
    leftmotor.restoreFactoryDefaults();

    rightmotor.setIdleMode(IdleMode.kBrake);
    leftmotor.setIdleMode(IdleMode.kBrake);

    // Current limiting to prevent frying the spark max
    rightmotor.setSmartCurrentLimit(80);
    leftmotor.setSmartCurrentLimit(80);

    rightmotor.follow(leftmotor, true);

    leftForwardLimitSwitch =
      leftmotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    leftReverseLimitSwitch =
      leftmotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    leftForwardLimitSwitch.enableLimitSwitch(true);
    leftReverseLimitSwitch.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    leftmotor.set(speed);
  }

  public void setVoltage(double voltage) {
    leftmotor.setVoltage(voltage);
  }

  public double getAbsolutePosition() {
    return (
      (
        (
          (-1 * absEncoder.getAbsolutePosition()) +
          Constants.CommandConstants.Arm.AbsEncoderOffset +
          1
        ) %
        1.0
      ) *
      (2 * Math.PI)
    );
    //((-1*absEncoder.getAbsolutePosition())+Constants.CommandConstants.Arm.AbsEncoderOffset)*2*Math.PI;
    // return((-1*absEncoder.getAbsolutePosition())+Constants.CommandConstants.Arm.AbsEncoderOffset);

    // return absEncoder.getAbsolutePosition();
  }

  public boolean getArmStatusFw() {
    return leftForwardLimitSwitch.isPressed();
  }

  public boolean getArmStatusRv() {
    return leftReverseLimitSwitch.isPressed();
  }
}
