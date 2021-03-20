package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

import static com.spartronics4915.frc2020.Constants.Intake.*;

/**
 * The Intake subsystem takes balls from the playing field and outputs them to
 * storage.
 */
public class Intake extends SpartronicsSubsystem
{
    private SpartronicsMotor mHarvestMotor;

    public Intake()
    {
        mHarvestMotor = SpartronicsSRX.makeMotor(kHarvestMotorId);
        if (mHarvestMotor.hadStartupError())
        {
            mHarvestMotor = new SpartronicsSimulatedMotor(kHarvestMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }

        mHarvestMotor.setBrakeMode(false);
        stop();
        mHarvestMotor.setOutputInverted(true);
    }

    /**
     * Activates the mechanum "vector" wheels in intake
     */
    public void harvest()
    {
        dashboardPutString("Status", "harvesting");
        mHarvestMotor.setPercentOutput(kHarvestSpeed);
    }

    /**
     * Reverses the vector wheel intake
     */
    public void reverse()
    {
        dashboardPutString("Status", "ejecting");
        mHarvestMotor.setPercentOutput(kEjectSpeed);
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
