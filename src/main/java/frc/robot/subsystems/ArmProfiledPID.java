// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmProfiledPID extends ProfiledPIDSubsystem{
    private final Arm arm;
    private final ArmFeedforward m_feedforward = new ArmFeedforward(1,0.90,2.49,0.07);

    public ArmProfiledPID(
        Arm arm
        ){
        // Input parameters from Constants
        // Start arm at rest in neutral position.
        super(
            new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                3, 10)),0
        );
        this.arm = arm;
        arm.getAbsolutePosition();
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
        return  arm.getAbsolutePosition();
    }
}
