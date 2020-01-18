package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public class ExampleSubsystem extends SpartronicsSubsystem {

    public ExampleSubsystem() {
        // Construct your hardware here
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

    public boolean ballLoaded() {
        return false;
    }

    // The exception to this is a general-functionality stop() method.
}
