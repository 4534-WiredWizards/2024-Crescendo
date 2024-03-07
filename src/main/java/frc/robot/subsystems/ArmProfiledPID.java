// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmProfiledPID extends ProfiledPIDSubsystem{
    private final Arm arm;
    // private final ArmFeedforward m_feedforward = new ArmFeedforward(1,0.45,2.49,0.03);
    
    // OLD WITH HEAVY ARM - NO SHOCKS
    private final ArmFeedforward m_feedforward = new ArmFeedforward(1,1.09,2.49,0.08);

    public ArmProfiledPID(
        Arm arm
        ){
        // Input parameters from Constants
        // Start arm at rest in neutral position.
        super(
            new ProfiledPIDController(
            4.2,
            4.1,
            0,
            new TrapezoidProfile.Constraints(5, 1)),0
        );
        this.getController().setTolerance(Units.degreesToRadians(2), 2);
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
