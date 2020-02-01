package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;

/**
 * This subsystem has two motors. A NEO using a Spark, while the other is a 775 PRO using a Talon.
 * The NEO motor winches the climber and the 775 PRO extends the climber
 * The four methods used are extend(), winch(), reverseExtend(), and stop()
 */
public class Climber extends SpartronicsSubsystem
{
    private final TalonSRX mLiftMotor;
    private final CANSparkMax mWinchMotor;

    public Climber()
    {
        // Hardware Contructor (Add motors and such here when I get them)
        mLiftMotor = new TalonSRX(Constants.Climber.kLiftMotorId);
        mWinchMotor = new CANSparkMax(Constants.Climber.kWinchMotorId, MotorType.kBrushless);
    }

    public void extend()
    {
        mLiftMotor.set(ControlMode.PercentOutput, Constants.Climber.kExtendSpeed);
        mWinchMotor.set(0.0);
    }

    public void winch(boolean stalled)
    {
        mLiftMotor.set(ControlMode.PercentOutput, 0.0);
        if (stalled)
            mWinchMotor.set(Constants.Climber.kWinchSpeed);
        else
            mWinchMotor.set(-Constants.Climber.kWinchSpeed);
    }

    public void retract()
    {
        mLiftMotor.set(ControlMode.PercentOutput, -Constants.Climber.kExtendSpeed);
        mWinchMotor.set(0.0);
    }

    public void stop()
    {
        mLiftMotor.set(ControlMode.PercentOutput, 0.0);
        mWinchMotor.set(0.0);
    }

    public double getWinchVoltage()
    {
        return mWinchMotor.getBusVoltage();
    }

    public boolean isStalled()
    {
        return false;
    }
}
