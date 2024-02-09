// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooser extends SubsystemBase {

  public enum AutoMode{

  }

  private SendableChooser<AutoMode> autoChooser;
  private Command autoRoutine;
  /** Creates a new AutoChooser. */
  public AutoChooser() {
    autoChooser = new SendableChooser<AutoMode>();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAuto(){
    AutoMode selectedAutoMode = (AutoMode) (autoChooser.getSelected());
    System.out.println("Running getAuto");
    switch (selectedAutoMode) {
  }
  return autoRoutine;
}
}

