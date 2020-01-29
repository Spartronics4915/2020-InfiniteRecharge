package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SpartronicsSubsystem {
    private double currentPosition = 0;

    private SpartronicsMax mSpinner; // Spins indexer.
    private SpartronicsSRX mLoader; // Loads ball into shooter

    private SensorModel mSpinnerModel;
    private SensorModel mLoaderModel;

    private DigitalInput mOpticalFlag;
    private DigitalInput mProxSensor;
    
    public Indexer() {
        // Set Spinner
        mSpinnerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mSpinner = new SpartronicsMax(Constants.Indexer.Spinner.kMotorId, mSpinnerModel);
        // Set up values
        mSpinner.setVelocityGains(Constants.Indexer.Spinner.kVelocityP, Constants.Indexer.Spinner.kVelocityD);
        mSpinner.setPositionGains(Constants.Indexer.Spinner.kPositionP, Constants.Indexer.Spinner.kPositionD);
        
        // Set Loader
        mLoaderModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mLoader = new SpartronicsSRX(Constants.Indexer.Loader.kMotor, mLoaderModel);
        // Set up values
        mLoader.setVelocityGains(Constants.Indexer.Loader.kVelocityP, Constants.Indexer.Loader.kVelocityD);
        mLoader.setPositionGains(Constants.Indexer.Loader.kPositionP, Constants.Indexer.Loader.kPositionD);

        // Setup Optical Flag for zeroing position
        mOpticalFlag = new DigitalInput(Constants.Indexer.kOpticalFlagId);

        // Setup Prox Sensor for indexing
        mProxSensor = new DigitalInput(Constants.Indexer.kProxSensorId);
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

    public boolean checkFlag() { // Checks whether the optical flag is triggered.
        return (Constants.Indexer.kOpticalFlagReversed ? mOpticalFlag.get() : !mOpticalFlag.get());
    }

    public void spinAt(double velocity) { // Spins motor at velocity
        mSpinner.setVelocity(velocity);
    }

    public void setZero() {
        mSpinner.getEncoder().setPosition(0);
    }

    public boolean getBallLoaded() { // Checks if ball is loaded
        return mProxSensor.get();
    }

    public void rotateN(int N) { // perform N quarter-rotations
        double targetPosition = 0.25 * ((double) N); // Cast N to double and convert to rotations
        currentPosition += targetPosition;
        mSpinner.setPosition(currentPosition);       // Rotate Spinner to target.
    }

    public void launch() { // Loads balls into shooter
        mLoader.setVelocity(Constants.Indexer.Loader.kSpeed);
    }

    public void stopLaunch() {
        mLoader.setVelocity(0);
    }

    // The exception to this is a general-functionality stop() method.

    public void stop() {
        mLoader.setNeutral();
        mSpinner.setNeutral();
    }
}
