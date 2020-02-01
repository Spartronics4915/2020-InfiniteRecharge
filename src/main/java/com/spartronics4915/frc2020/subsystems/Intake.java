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
    private SpartronicsMotor mIngestMotor;

    /** constructor **/
    public Intake()
    {
        mHarvestMotor = SpartronicsSRX.makeMotor(Constants.Intake.kHarvestMotorId, SensorModel.fromMultiplier(1));
        mIngestMotor = SpartronicsSRX.makeMotor(Constants.Intake.kIngestMotorId, SensorModel.fromMultiplier(1));
        if (mHarvestMotor.hadStartupError() || mIngestMotor.hadStartupError())
        {
            mHarvestMotor = new SpartronicsSimulatedMotor();
            mIngestMotor = new SpartronicsSimulatedMotor();
            logInitialized(false);
        } else {
            logInitialized(true);
        }
    }

    /** 
     * activates the mechanum "vector" wheels in partial intake
     * hopefully in tandem with the prism roller 
    **/
    public void harvestIntake() 
    {
        mHarvestMotor.setDutyCycle(Constants.Intake.kHarvestSpeed);
    }

    /** 
     * in partial intake
     * hopefully in tandem with the mechanum "vector" wheels 
    **/
    public void ingestIntake() 
    {
        mIngestMotor.setDutyCycle(Constants.Intake.kIngestSpeed);
    }

    /** reverses vector wheels **/
    public void harvestReverse() 
    {
        mHarvestMotor.setDutyCycle(-Constants.Intake.kHarvestSpeed);
    }

    /** reverses prism roller **/
    public void ingestReverse() 
    {
        mIngestMotor.setDutyCycle(-Constants.Intake.kIngestSpeed);
    }

    /** stops vector wheels **/
    public void harvestStop() 
    {
        mHarvestMotor.setDutyCycle(0.0);
    }

    /** stops prism roller **/
    public void ingestStop() 
    {
        mIngestMotor.setDutyCycle(0.0);
    }

    /** checks to see if ball is held in intake chamber **/
    public void isBallHeld() 
    {

    }

    /** universal stop method **/
    public void stop()
    {
        mHarvestMotor.setNeutral();
        mIngestMotor.setNeutral();
    }
}
