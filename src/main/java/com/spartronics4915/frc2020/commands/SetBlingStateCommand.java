package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.frc2020.subsystems.LED;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The method to switch the BlingState on the Arduino.
 */
public class SetBlingStateCommand extends CommandBase
{
    private LED mLED;
    private BlingState mBlingState;

    public SetBlingStateCommand(LED led_system, BlingState blingState)
    {
        mLED = led_system;
        addRequirements(mLED);
        mBlingState = blingState;
    }

    @Override
    public void initialize()
    {
        Logger.info("SetBlingStateCommand initialized: " + mBlingState.toString());
        mLED.setBlingState(mBlingState);
    }

    @Override
    public void execute()
    {
        // Doesn't need to do anything
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean isInterrupted)
    {
    }
}
