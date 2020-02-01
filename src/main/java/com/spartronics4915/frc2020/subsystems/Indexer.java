package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Indexer for storing power cells
 */
public class Indexer extends SpartronicsSubsystem
{
    private double targetPosition = 0;

    private SpartronicsMotor mSpinnerMotor;
    private SpartronicsMotor mLoaderMotor;

    private SensorModel mSpinnerModel;
    private SensorModel mLoaderModel;

    private DigitalInput mOpticalFlag;
    private DigitalInput mProxSensor;

    private boolean mIsLaunching = false;

    public Indexer()
    {
        // Set up Spinner
        mSpinnerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mSpinnerMotor = SpartronicsMax.makeMotor(Constants.Indexer.Spinner.kMotorId, mSpinnerModel);
        // Set up gains
        mSpinnerMotor.setVelocityGains(Constants.Indexer.Spinner.kVelocityP,
            Constants.Indexer.Spinner.kVelocityD);
        mSpinnerMotor.setPositionGains(Constants.Indexer.Spinner.kPositionP,
            Constants.Indexer.Spinner.kPositionD);

        // Set up Loader
        mLoaderModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mLoaderMotor = SpartronicsSRX.makeMotor(Constants.Indexer.Loader.kMotor, mLoaderModel);
        // Set up gains
        mLoaderMotor.setVelocityGains(Constants.Indexer.Loader.kVelocityP,
            Constants.Indexer.Loader.kVelocityD);
        mLoaderMotor.setPositionGains(Constants.Indexer.Loader.kPositionP,
            Constants.Indexer.Loader.kPositionD);

        // Setup Optical Flag for zeroing position
        mOpticalFlag = new DigitalInput(Constants.Indexer.kOpticalFlagId);

        // Setup Proximity Sensor for indexing
        mProxSensor = new DigitalInput(Constants.Indexer.kProxSensorId);
    }

    /**
     * @return Whether or not the optical flag is triggered.
     */
    public boolean checkFlag()
    {
        return (Constants.Indexer.kOpticalFlagReversed ? mOpticalFlag.get() : !mOpticalFlag.get());
    }

    /**
     * Sets the spinner to a specific velocity
     * @param velocity the velocity to spin the spinner at
     */
    public void spinAt(double velocity)
    {
        mSpinnerMotor.setVelocity(velocity);
    }

    /**
     * Sets the spinner encoder to zero at it's current position
     */
    public void setZero()
    {
        mSpinnerMotor.getEncoder().setPosition(0);
    }

    /**
     * @return whether or not the ball is loaded in the first slot
     */
    public boolean getBallLoaded()
    {
        return mProxSensor.get();
    }

    /**
     * Rotate the spinner a certain amount of rotations
     * @param N the number of quarter rotations to perform
     */
    public void rotateN(int N)
    {
        double deltaPosition = 0.25 * ((double) N); // Cast N to double and convert to rotations
        targetPosition += deltaPosition;
        mSpinnerMotor.setPosition(targetPosition); // Rotate Spinner to target.
    }

    /**
     * Returns spinner to "0" position on the encoder
     */
    public void returnToHome()
    {
        targetPosition = 0;
        mSpinnerMotor.setPosition(targetPosition);
    }

    /**
     * Move spinner to nearest position
     */
    public void toNearestQuarterRotation()
    {
        // Rotates to nearest quarter rotation
        targetPosition = Math.ceil(mSpinnerMotor.getEncoder().getPosition() * 4) / 4;
        mSpinnerMotor.setPosition(targetPosition);
    }

    /**
     * Start loading balls into the shooter
     */
    public void launch()
    {
        mIsLaunching = true;
        mLoaderMotor.setVelocity(Constants.Indexer.Loader.kSpeed);
    }

    /**
     * Stop loading balls into the shooter
     */
    public void endLaunch()
    {
        mIsLaunching = false;
        mLoaderMotor.setVelocity(0);
    }

    /**
     * @return whether or not the loader motor is running
     */
    public boolean isLaunching()
    {
        return mIsLaunching;
    }

    /**
     * Stop all motors
     */
    public void stop()
    {
        mLoaderMotor.setNeutral();
        mSpinnerMotor.setNeutral();
    }

    /**
     * Stop Spinner
     */
    public void stopSpinner()
    {
        mSpinnerMotor.setNeutral();
    }
}
