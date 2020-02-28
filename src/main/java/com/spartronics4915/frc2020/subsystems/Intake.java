package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

/**
 * The Intake subsystem takes balls from the playing field and outputs them to
 * storage.
 */
public class Intake extends SpartronicsSubsystem
{
    private SpartronicsMotor mHarvestMotor;

    public Intake()
    {
        mHarvestMotor = SpartronicsSRX.makeMotor(Constants.Intake.kHarvestMotorId);
        if (mHarvestMotor.hadStartupError())
        {
            mHarvestMotor = new SpartronicsSimulatedMotor(Constants.Intake.kHarvestMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }

        stop();
        mHarvestMotor.setOutputInverted(true);
    }

    /**
     * Activates the mechanum "vector" wheels in intake
     */
    public void harvest()
    {
        dashboardPutString("Status", "harvesting");
        mHarvestMotor.setPercentOutput(Constants.Intake.kHarvestSpeed);
    }

    /**
     * Reverses the vector wheel intake
     */
    public void reverse()
    {
        dashboardPutString("Status", "ejecting");
        mHarvestMotor.setPercentOutput(Constants.Intake.kEjectSpeed);
    }

    /**
     * Universal stop method
     */
    public void stop()
    {
        dashboardPutString("Status", "stopped");
        mHarvestMotor.setNeutral();
    }
}
