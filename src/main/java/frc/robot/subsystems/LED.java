// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LED extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.SubsystemConstants.CANdleID, "rio");


  /** Creates a new LED. */
  public LED() {}

  // Subsystem for controlling LED matrix panel and LED strips
  //  Uses ctre CANdle library to control LED strips

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
