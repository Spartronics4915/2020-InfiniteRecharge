package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

/**
 * The Popper subsystem controls the omniwheel used to kick balls up into the launcher.
 * <p>
 * Available as a separate subsystem for easy use of CommandGroups.
 */
public class Popper extends SpartronicsSubsystem
{
    private SpartronicsMotor mPopperMotor;
    private SensorModel mPopperModel;

    private boolean mIsLaunching = false;

    public Popper()
    {
        mPopperModel = SensorModel.fromMultiplier(Constants.Popper.kConversionRatio);
        mPopperMotor = SpartronicsSRX.makeMotor(Constants.Popper.kMotorId, mPopperModel);

        if (mPopperMotor.hadStartupError())
        {
            mPopperMotor = new SpartronicsSimulatedMotor(Constants.Popper.kMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }

        // Set up gains for loader
        mPopperMotor.setVelocityGains(Constants.Popper.kVelocityP,
            Constants.Popper.kVelocityD);
        mPopperMotor.setPositionGains(Constants.Popper.kPositionP,
            Constants.Popper.kPositionD);
    }

    /**
     * Start loading balls into the shooter
     */
    public void pop()
    {
        mIsLaunching = true;
        mPopperMotor.setVelocity(Constants.Popper.kSpeed);
    }

    /**
     * Stop loading balls into the shooter
     */
    public void stop()
    {
        mIsLaunching = false;
        mPopperMotor.setVelocity(0);
    }

    /**
     * @return whether or not the loader motor is running
     */
    public boolean isLaunching()
    {
        return mIsLaunching;
    }
}
