// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ArmProfiledPID extends ProfiledPIDSubsystem{
    private final Arm arm;

    // ARM CONSTANTS
    private final ArmFeedforward m_feedforward = new ArmFeedforward(Constants.CommandConstants.Arm.kSVolts,
    Constants.CommandConstants.Arm.kGVolts,Constants.CommandConstants.Arm.kVVoltSecondPerRad,Constants.CommandConstants.Arm.kAVoltSecondSquaredPerRad);

    public ArmProfiledPID(
        Arm arm
        ){
        // Input parameters from Constants
        // Start arm at rest in neutral position.
        super(
            new ProfiledPIDController(
                6,//3.1 // Down from 6
                4.1,//3.4 // Down from 4.1
                0,
            new TrapezoidProfile.Constraints(9, 1))
        );
        this.getController().setTolerance(Units.degreesToRadians(2), 1);
        this.arm = arm;
        // arm.getAbsolutePosition();
        // Input goal, rather self explanatory: Constants.kArmOffsetRads
        setGoal(0);
    }

     @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        arm.setVoltage(output + feedforward);
    }   

  @Override
    public double getMeasurement() {
        // abs_Encoder.getDistance() + ArmConstants.kArmOffsetRads
        // Debug
        // System.out.println("getMeasurement:"+arm.getAbsolutePosition());
        return arm.getAbsolutePosition();
    }


    // At goal from ProfilePIDSubsystem
    public boolean atPIDGoal(){
        return this.getController().atGoal();
    }
}
