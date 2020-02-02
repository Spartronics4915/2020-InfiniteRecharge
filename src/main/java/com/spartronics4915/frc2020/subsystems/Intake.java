package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

/**
 * The Intake subsystem takes balls from
 * the playing field and outputs them to storage.
 */
public class Intake extends SpartronicsSubsystem
{
    private SpartronicsMotor mHarvestMotor;

    /** constructor **/
    public Intake()
    {
        mHarvestMotor = SpartronicsSRX.makeMotor(Constants.Intake.kHarvestMotorId,
            SensorModel.fromMultiplier(1));
        if (mHarvestMotor.hadStartupError())
        {
            mHarvestMotor = new SpartronicsSimulatedMotor();
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
    }

    /**
     * activates the mechanum "vector" wheels in partial intake
     * hopefully in tandem with the prism roller
    **/
    public void harvest()
    {
        mHarvestMotor.setDutyCycle(Constants.Intake.kHarvestSpeed);
    }

    /** reverses vector wheels **/
    public void reverse()
    {
        mHarvestMotor.setDutyCycle(Constants.Intake.kHarvestReverseSpeed);
    }

    /** checks to see if ball is held in intake chamber **/
    public void isBallHeld()
    {

    }

    /** universal stop method **/
    public void stop()
    {
        mHarvestMotor.setNeutral();
    }
}
