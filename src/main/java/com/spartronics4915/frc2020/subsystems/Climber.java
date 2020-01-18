package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.frc2020.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Objective:
 * Subsystem will provide necessary controls/APIs for climbing consistently. Use cases:
 * 1. Accurately hitting the bar in order to climb with upmost efficiancy
 * 2. Describing if the climber is functional or not during the match
 * 3. Not causing any mechanical issues when climbing (battery falling out and the like)
 * 
 * Integrations:
 * 
 */
public class Climber extends SpartronicsSubsystem {
    private static TalonSRX mClimber775PRO;
    private static CANSparkMax mClimberNEO;
    public Climber() {
        //Hardware Contructor (Add motors and such here when I get them)
        mClimber775PRO = new TalonSRX(Constants.kClimber775ProID);
        mClimberNEO = new CANSparkMax(Constants.kClimberNEOID, MotorType.kBrushless);
        
    }
    
    public static void extend() {
        mClimber775PRO.set(ControlMode.PercentOutput, 0.0);
        mClimberNEO.set(0.0);
    }

    public static void winch() {
        mClimber775PRO.set(ControlMode.PercentOutput, 0.0);
        mClimberNEO.set(0.0);
    }

    public static void reverse() {
        mClimber775PRO.set(ControlMode.PercentOutput, 0.0);
        mClimberNEO.set(0.0);
    }

    public static void stop() {
        mClimber775PRO.set(ControlMode.PercentOutput, 0.0);
        mClimberNEO.set(0.0);
    }

}
