package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public class Indexer extends SpartronicsSubsystem {

    public int mSpinnerId = -1;
    public int mLifterId = -1;

    public Indexer() {
        // Construct your hardware here
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

    public boolean getBallLoaded() {
        return false;
    }

    public void rotateClockwiseOnce() {

    }

    public void rotateCounterClockwiseOnce() {

    }

    public int rotateCounterClockwiseFull() {
        return 0;
    }

    public int rotateClockwiseFull() {
        return 0;
    }

    public int rotateFull() {}
        return 0;
    }

    public int rotateCounterClockwiseEmpty() {
        return 0;
    }

    public int rotateClockwiseEmpty() {
        return 0;
    }

    public int rotateEmpty() {
        return 0;
    }

    public void stop() 
    {

    }

    public void liftUp()
    {

    }

    public void liftDown()
    {

    }

    // The exception to this is a general-functionality stop() method.
}
