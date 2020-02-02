package com.spartronics4915.frc2020.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAbsoluteEncoderCommand extends CommandBase
{
    private boolean reverse = false;
    private int n = 0;
    private double pos = 0;
    SpartronicsMotor mMotor;
    SensorModel mMotorModel;

    // You should only use one subsystem per command. If multiple are needed, use a
    // CommandGroup.
    public TestAbsoluteEncoderCommand()
    {
        mMotorModel = SensorModel.toDegrees(4096);
        mMotor = SpartronicsSRX.makeMotor(2, mMotorModel, FeedbackDevice.CTRE_MagEncoder_Absolute);
        pos = mMotor.getEncoder().getPosition();
        mMotor.setPositionGains(1, 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        mMotor.setPosition(70);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        Logger.info("Position:" + mMotor.getEncoder().getPosition() + " (" + (((int) mMotor.getEncoder().getPosition()) % 360) + ") Target: " + pos*/);
        
        
        // mMotor.setPosition(pos);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        mMotor.setNeutral();
    }
}
