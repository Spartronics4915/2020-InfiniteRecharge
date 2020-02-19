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
 * 1) A NEO motor used to winch the climber
 * 2) A 775 PRO motor used to lift the climber
 *
 * USE CASE 1: CLIMB FROM THE GROUND TO BAR THAT IS LEVEL
 * 1) Press and hold a single button to raise the climber to the desired level
 * 2) Hit the winch button and watch the magic happen
 *
 * USE CASE 2: CLIMBER FROM THE GROUND TO BAR THAT IS ELEVATED
 * 1) Press the Climb to Max button
 * 2) Winch
 *
 * USE CASE 3: CLIMB FROM THE GROUND TO BAR THAT IS SUNKEN
 * 1) Press Climb to Min button
 * 2) Winch
 *
 * USE CASE 4: RETRACT
 * 1) Press the retract button
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
        mLiftMotor.setPercentOutput(Constants.Climber.kExtendSpeed);
        mWinchMotor.setPercentOutput(0.0);
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
        mLiftMotor.setPercentOutput(0.0);
        if (stalled)
            mWinchMotor.setPercentOutput(Constants.Climber.kWinchSpeed);
        else
            mWinchMotor.setPercentOutput(Constants.Climber.kReverseWinchSpeed);
    }

    /**
     * Lowers the "lightsaber"
     */
    public void retract()
    {
        mLiftMotor.setPercentOutput(Constants.Climber.kRetractSpeed);
        mWinchMotor.setPercentOutput(0.0);
    }

    /**
     * Universal stop method
     */
    public void stop()
    {
        mLiftMotor.setPercentOutput(0.0);
        mWinchMotor.setPercentOutput(0.0);
    }

    /**
     * @return Whether the output current of the Winch motor is above the "stall" threshold
     */
    public boolean isStalled()
    {
        return mWinchMotor.getOutputCurrent() >= Constants.Climber.kStallThreshold;
    }

    public boolean secondaryIsStalled()
    {
        return mWinchMotor.getOutputCurrent() >= Constants.Climber.kSecondaryStallThreshold;
    }

    private double mMaxAmps = Double.NEGATIVE_INFINITY;

    @Override
    public void periodic()
    {
        double amps = mWinchMotor.getOutputCurrent();
        if (amps > mMaxAmps)
        {
            mMaxAmps = amps;
            dashboardPutNumber("winchCurrentMax", mMaxAmps);
        }
        dashboardPutNumber("winchCurrent", mMaxAmps);
    }
}
