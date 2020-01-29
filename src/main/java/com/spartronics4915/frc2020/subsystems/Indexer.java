package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SpartronicsSubsystem {

    private SpartronicsMax mSpinner;
    private SpartronicsSRX mLoader;

    private SensorModel mSpinnerModel;
    private SensorModel mLoaderModel;

    private DigitalInput mOpticalFlag;

    public Indexer() {
        // Set Spinner
        mSpinnerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mSpinner = new SpartronicsMax(Constants.Indexer.Spinner.kMotorId, mSpinnerModel);
        mSpinner.setVelocityGains(Constants.Indexer.Spinner.kP, Constants.Indexer.Spinner.kD);
        
        // Set Loader
        mLoaderModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mLoader = new SpartronicsSRX(Constants.Indexer.Loader.kMotor, mLoaderModel);

        // Setup Optical Flag
        mOpticalFlag = new DigitalInput(Constants.Indexer.kOpticalFlagId);
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

    public void zeroSlow() {
        mSpinner.setVelocity(0.1);
    }

    public boolean getBallLoaded() {
        return false;
    }

    public void rotateOnce() {
        
    }

    public int rotateFull() {
        return 0;
    }

    public int rotateEmpty() {
        return 0;
    }

    public void loadBall() {

    }

    public void unloadBall() {
        
    }

    // The exception to this is a general-functionality stop() method.

    public void stop() {
        
    }
}
