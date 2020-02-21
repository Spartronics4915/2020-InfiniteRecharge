package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.LED;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;

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
     * Changing bling state is a write & forget process
     */
    public class SetBlingState extends InstantCommand
    {
        public SetBlingState(BlingState blingState)
        {
            super(() -> { mLED.setBlingState(blingState); }, mLED);
        }
    }

}
