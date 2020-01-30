package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Indexer for storing power cells
 */
public class Indexer extends SpartronicsSubsystem
{
    private double targetPosition = 0;

    private SpartronicsMax mSpinner; // Spins indexer.
    private SpartronicsSRX mLoader; // Loads ball into shooter

    private SensorModel mSpinnerModel;
    private SensorModel mLoaderModel;

    private DigitalInput mOpticalFlag;
    private DigitalInput mProxSensor;

    private boolean mIsLaunching = false;

    public Indexer()
    {
        // Set up Spinner
        mSpinnerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mSpinner = new SpartronicsMax(Constants.Indexer.Spinner.kMotorId, mSpinnerModel);
        // Set up gains
        mSpinner.setVelocityGains(Constants.Indexer.Spinner.kVelocityP,
                Constants.Indexer.Spinner.kVelocityD);
        mSpinner.setPositionGains(Constants.Indexer.Spinner.kPositionP,
                Constants.Indexer.Spinner.kPositionD);

        // Set up Loader
        mLoaderModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mLoader = new SpartronicsSRX(Constants.Indexer.Loader.kMotor, mLoaderModel);
        // Set up gains
        mLoader.setVelocityGains(Constants.Indexer.Loader.kVelocityP,
                Constants.Indexer.Loader.kVelocityD);
        mLoader.setPositionGains(Constants.Indexer.Loader.kPositionP,
                Constants.Indexer.Loader.kPositionD);

        // Setup Optical Flag for zeroing position
        mOpticalFlag = new DigitalInput(Constants.Indexer.kOpticalFlagId);

        // Setup Prox Sensor for indexing
        mProxSensor = new DigitalInput(Constants.Indexer.kProxSensorId);
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

    /**
     * 
     * @return Whether or not the optical flag is triggered.
     */
    public boolean checkFlag()
    { // Checks whether the optical flag is triggered.
        return (Constants.Indexer.kOpticalFlagReversed ? mOpticalFlag.get() : !mOpticalFlag.get());
    }

    /**
     * Sets the spinner to a specific velocity
     * @param velocity the velocity to spin the spinner at
     */
    public void spinAt(double velocity)
    { // Spins motor at velocity
        mSpinner.setVelocity(velocity);
    }

    /**
     * Sets the spinner encoder to zero at it's current position
     */
    public void setZero()
    {
        mSpinner.getEncoder().setPosition(0);
    }

    /**
     * @return whether or not the ball is loaded in the first slot
     */
    public boolean getBallLoaded()
    { // Checks if ball is loaded
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
        mSpinner.setPosition(targetPosition); // Rotate Spinner to target.
    }

    /**
     * Returns spinner to "0" position on the encoder
     */
    public void returnSpin()
    {
        targetPosition = 0;
        mSpinner.setPosition(targetPosition);
    }

    /**
     * Move spinner to nearest position
     */
    public void endSpinner()
    {
        // Rotates to nearest quarter rotation
        targetPosition = Math.ceil(mSpinner.getEncoder().getPosition() * 4) / 4;
        mSpinner.setPosition(targetPosition);
    }

    /**
     * Start loading balls into the shooter
     */
    public void load()
    { // Loads balls into shooter
        mIsLaunching = true;
        mLoader.setVelocity(Constants.Indexer.Loader.kSpeed);
    }

    /**
     * Stop loading balls into the shooter
     */
    public void endLaunch()
    {
        mIsLaunching = false;
        mLoader.setVelocity(0);
    }

    /**
     * 
     * @return whether or not the loader motor is running
     */
    public boolean getLaunching()
    {
        return mIsLaunching;
    }

    // The exception to this is a general-functionality stop() method.

    /**
     * Stop all motors
     */
    public void stop()
    {
        mLoader.setNeutral();
        mSpinner.setNeutral();
    }

    /**
     * Stop Spinner
     */
    public void stopSpinner()
    {
        mSpinner.setNeutral();
    }
}
