package com.spartronics4915.lib.hardware.motors;

<<<<<<< HEAD
public interface SpartronicsEncoder
{
=======
import com.spartronics4915.lib.util.Logger;

public interface SpartronicsEncoder {
>>>>>>> 1c015490772d15780e2e59b00e44b886fac47e35

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
