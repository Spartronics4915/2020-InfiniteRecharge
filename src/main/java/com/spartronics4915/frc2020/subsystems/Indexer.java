package com.spartronics4915.frc2020.subsystems;

// import com.spartronics4915.frc2020.Constants;
// import com.spartronics4915.lib.hardware.motors.SensorModel;
// import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
// import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public class Indexer extends SpartronicsSubsystem {

    // private SpartronicsMax mSpinner;
    // private SpartronicsSRX mLoader;

    // private SensorModel mSpinnerModel;
    // private SensorModel mLoaderModel;

    public Indexer() {
        // mSpinnerModel = SensorModel
        // mSpinner = new SpartronicsMax(Constants.Indexer.kSpinnerId);
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.

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
