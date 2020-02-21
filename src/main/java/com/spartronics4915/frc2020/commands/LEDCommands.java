package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.LED;
import com.spartronics4915.frc2020.subsystems.LED.Bling;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Bling animations
 * The method to switch the BlingState on the Arduino
 */
public class LEDCommands
{
    private final LED mLED;

    public LEDCommands(LED led)
    {
        mLED = led;
        // Note: LED does NOT have a default command
        // Bling states are activated based on joystick triggers OR robot states
    }

    /**
     * This {@ InstantCommand} schedules the bling state change request
     * Changing bling state is a write & forget process.  Note that
     * we don't utilized the standard constructor and instead override
     * the initialize method.  This is because we can't access mLED in
     * a runnable being passed to the standard super constructor.
     */
    public class SetBlingState extends InstantCommand
    {
        Bling mState;
        public SetBlingState(Bling s)
        {
            super();
            this.addRequirements(mLED);
            this.mState = s;
        }
        @Override
        public void initialize()
        {
            mLED.setBlingState(this.mState);
        }
    }

}
