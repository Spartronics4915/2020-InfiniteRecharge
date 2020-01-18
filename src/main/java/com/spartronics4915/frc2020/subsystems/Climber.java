package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Climber extends SpartronicsSubsystem {

    private TalonSRX mClimber775PRO;
    private CANSparkMax mClimberNEO;
    public Climber() {
        //Hardware Contructor (Add motors and such here when I get them)
        mClimber775PRO = new TalonSRX(5);
        mClimberNEO = new CANSparkMax(6, MotorType.kBrushless);
        
    }
    
    public static void extend() {
        
    }

    public static void winch() {

    }

    public static void reverse() {

    }

    public static void stop() {

    }

}
