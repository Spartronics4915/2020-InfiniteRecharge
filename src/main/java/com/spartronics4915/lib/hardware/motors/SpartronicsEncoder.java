package com.spartronics4915.lib.hardware.motors;

import com.spartronics4915.lib.util.Logger;

public interface SpartronicsEncoder {

    /**
     * @return Velocity in meters/second.
     */
    double getVelocity();

    /**
     * @return Position in meters.
     */
    double getPosition();

    /**
     * Sets the "direction" (phase) of this encoder.
     * 
     * @param isReversed If true, the sensor's output is reversed.
     */
    void setPhase(boolean isReversed);
}