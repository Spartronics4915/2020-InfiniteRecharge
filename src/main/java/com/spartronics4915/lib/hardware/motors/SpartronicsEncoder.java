package com.spartronics4915.lib.hardware.motors;

import com.spartronics4915.lib.util.Logger;

public interface SpartronicsEncoder
{

    public final SpartronicsEncoder kDisconnectedEncoder = new SpartronicsEncoder()
    {

        @Override
        public void setPhase(boolean isReversed)
        {
            Logger.error("Couldn't set the phase of a disconnected encoder");
        }

        @Override
        public double getVelocity()
        {
            return 0;
        }

        @Override
        public double getPosition()
        {
            return 0;
        }

        @Override
        public void setPosition(double position)
        {
            Logger.error("Couldn't set the position of a disconnected encoder");
        }
    };

    /**
     * @return Velocity in custom units/second.
     */
    double getVelocity();

    /**
     * @return Position in custom units.
     */
    double getPosition();

    /**
     * Sets the "direction" (phase) of this encoder.
     * 
     * @param isReversed If true, the sensor's output is reversed.
     */
    void setPhase(boolean isReversed);

    /**
     * Sets the current position (The value stored, not PID target)
     * 
     * @param TargetPosition position
     */
    void setPosition(double position);
}
