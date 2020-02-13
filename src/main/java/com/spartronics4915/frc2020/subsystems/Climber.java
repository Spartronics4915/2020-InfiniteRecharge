package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;

/**
 * CLIMBER
 * 
 * OBJECTIVE: Be able to grab and hold on to the shield generator
 * and remain balanced there even after the robot is deactiviated
 * Use cases:
 * 1) Climb from the ground to bar that is level, keeping it level in the process
 * 2) Climb from the ground to bar that is elevated, leveling it in the process
 * 3) Climb from the ground to bar that is sunken, staying off the ground if possible
 * 4) Able to reverse the extend function for repositioning purposes
 * 
 * INTEGRATIONS:
 * 1) A NEO motor used to 
 * 2) A 775 PRO motor used to 
 * 
 */
public class Climber extends SpartronicsSubsystem
{
    private SpartronicsMotor mLiftMotor;
    private SpartronicsMotor mWinchMotor;

    public Climber()
    {
        // Hardware Contructor (Add motors and such here when I get them)
        mLiftMotor = SpartronicsSRX.makeMotor(Constants.Climber.kLiftMotorId);
        mWinchMotor = SpartronicsMax.makeMotor(Constants.Climber.kWinchMotorId);

        if (mLiftMotor.hadStartupError() || mWinchMotor.hadStartupError())
        {
            mLiftMotor = new SpartronicsSimulatedMotor(Constants.Climber.kLiftMotorId);
            mWinchMotor = new SpartronicsSimulatedMotor(Constants.Climber.kWinchMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
    }

    /**
     * Extends the "lightsaber"
     */
    public void extend()
    {
        mLiftMotor.setDutyCycle(Constants.Climber.kExtendSpeed);
        mWinchMotor.setDutyCycle(0.0);
    }

    /**
     * Takes a parameter that reverses the motor direction.
     * <p>
     * The design of the gearbox means that running the Winch motor in either direction
     * will still winch rope, making the {@link Climber} strictly one-way.
     *
     * @param stalled Whether the winch has stalled yet
     */
    public void winch(boolean stalled)
    {
        mLiftMotor.setDutyCycle(0.0);
        if (stalled)
            mWinchMotor.setDutyCycle(Constants.Climber.kWinchSpeed);
        else
            mWinchMotor.setDutyCycle(Constants.Climber.kReverseWinchSpeed);
    }

    /**
     * Lowers the "lightsaber"
     */
    public void retract()
    {
        mLiftMotor.setDutyCycle(Constants.Climber.kRetractSpeed);
        mWinchMotor.setDutyCycle(0.0);
    }

    /**
     * Universal stop method
     */
    public void stop()
    {
        mLiftMotor.setDutyCycle(0.0);
        mWinchMotor.setDutyCycle(0.0);
    }

    /**
     * @return Whether the output current of the Winch motor is above the "stall" threshold
     */
    public boolean isStalled()
    {
        return mWinchMotor.getOutputCurrent() >= Constants.Climber.kStallThreshold;
    }
}
