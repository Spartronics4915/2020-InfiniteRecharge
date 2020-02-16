package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Indexer for storing power cells
 */
public class Indexer extends SpartronicsSubsystem
{
    private double mTargetPosition = 0;

    private SpartronicsMotor mIndexerMotor;
    private SpartronicsMotor mLoaderMotor;
    private SpartronicsMotor mTransferMotor;

    private SensorModel mIndexerModel;
    private SensorModel mLoaderModel;
    private SensorModel mTransferModel;

    private DigitalInput mLimitSwitch;
    private DigitalInput mOpticalProxSensor;
    private DigitalInput mIntakeProxSensor;

    private boolean mIsLaunching = false;
    private boolean mIsTransferring = false;

    private int mBallsHeld = 0;

    public boolean mIsFull = false;

    public Indexer()
    {
        // Set up Spinner
        mIndexerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mIndexerMotor = SpartronicsMax.makeMotor(Constants.Indexer.Spinner.kMotorId, mIndexerModel);
        // Set up Loader
        mLoaderModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mLoaderMotor = SpartronicsSRX.makeMotor(Constants.Indexer.Loader.kMotorId, mLoaderModel);
        // Set up Transfer
        mTransferModel = SensorModel.fromMultiplier(Constants.Indexer.Transfer.kConversionRatio);
        mTransferMotor = SpartronicsMax.makeMotor(Constants.Indexer.Transfer.kMotorId, mTransferModel);

        if (mIndexerMotor.hadStartupError() || mLoaderMotor.hadStartupError() || mTransferMotor.hadStartupError())
        {
            mIndexerMotor = new SpartronicsSimulatedMotor(Constants.Indexer.Spinner.kMotorId);
            mLoaderMotor = new SpartronicsSimulatedMotor(Constants.Indexer.Loader.kMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        // Set up gains for spinner
        mIndexerMotor.setVelocityGains(Constants.Indexer.Spinner.kVelocityP,
            Constants.Indexer.Spinner.kVelocityD);
        mIndexerMotor.setPositionGains(Constants.Indexer.Spinner.kPositionP,
            Constants.Indexer.Spinner.kPositionD);
        mIndexerMotor.setMotionProfileCruiseVelocity(Constants.Indexer.Spinner.kMaxVelocity); // Set motion profile
        mIndexerMotor.setMotionProfileMaxAcceleration(Constants.Indexer.Spinner.kMaxAcceleration);
        mIndexerMotor.setUseMotionProfileForPosition(true);

        // Set up gains for loader
        mLoaderMotor.setVelocityGains(Constants.Indexer.Loader.kVelocityP,
            Constants.Indexer.Loader.kVelocityD);
        mLoaderMotor.setPositionGains(Constants.Indexer.Loader.kPositionP,
            Constants.Indexer.Loader.kPositionD);

        // Set up gains for transfer
        mTransferMotor.setVelocityGains(Constants.Indexer.Transfer.kVelocityP, Constants.Indexer.Transfer.kVelocityD);
        mTransferMotor.setPositionGains(Constants.Indexer.Transfer.kPositionP, Constants.Indexer.Transfer.kPositionD);

        // Setup Optical Flag for zeroing position
        mLimitSwitch = new DigitalInput(Constants.Indexer.kLimitSwitchId);

        // Setup Proximity Sensors for indexing
        mOpticalProxSensor = new DigitalInput(Constants.Indexer.kSlotProxSensorId);
        mIntakeProxSensor  = new DigitalInput(Constants.Indexer.kIntakeSensorId);
    }

    /**
     * @return Whether or not the optical flag is triggered.
     */
    public boolean checkFlag()
    {
        return mLimitSwitch.get();
    }

    /**
     * Sets the spinner to a specific velocity
     * @param velocity the velocity to spin the spinner at
     */
    public void spinAt(double velocity)
    {
        mIndexerMotor.setDutyCycle(velocity);
    }

    /**
     * Sets the spinner encoder to zero at it's current position
     */
    public void setZero()
    {
        mIndexerMotor.getEncoder().setPosition(0);
    }

    /**
     * @return whether or not a ball is loaded in the first slot
     */
    public boolean getSlotBallLoaded()
    {
        return mOpticalProxSensor.get();
    }

    /**
     * Checks to see if a ball is held in the intake chamber
     * with a proximity sensor returning a digital value.
     * <p>
     * The style of proximity sensor we use requires MANUAL calibration.
     *
     * @return Whether a ball is held
     */
    public boolean getIntakeBallLoaded()
    {
        return !mIntakeProxSensor.get();
    }

    /**
     * Rotate the spinner a certain amount of rotations
     * @param N the number of quarter rotations to perform
     */
    public void rotateN(double N)
    {
        if (N != 0)
        {
            double deltaPosition = 0.25 * N; // Cast N to double and convert to rotations
            mTargetPosition += deltaPosition;
            mIndexerMotor.setPosition(mTargetPosition); // Rotate Spinner to target.
        }
    }

    /**
     * Returns spinner to "0" position on the encoder
     */
    public void returnToHome()
    {
        mTargetPosition = 0;
        mIndexerMotor.setPosition(mTargetPosition);
    }

    /**
     * Move spinner to nearest position
     */
    public void toNearestQuarterRotation()
    {
        // Rotates to nearest quarter rotation
        mTargetPosition = Math.ceil(mIndexerMotor.getEncoder().getPosition() * 4) / 4;
        mIndexerMotor.setPosition(mTargetPosition);
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

    public void transfer()
    {
        mIsTransferring = true;
        mTransferMotor.setVelocity(Constants.Indexer.Transfer.kSpeed);
    }

    public void endTransfer()
    {
        mIsTransferring = false;
        mTransferMotor.setVelocity(0);
    }

    public boolean isTransferring()
    {
        return mIsTransferring;
    }

    /**
     * Stop all motors
     */
    public void stop()
    {
        mLoaderMotor.setNeutral();
        mIndexerMotor.setNeutral();
    }

    /**
     * Stop Spinner
     */
    public void stopSpinner()
    {
        mIndexerMotor.setNeutral();
    }

    public boolean isAtPosition()
    {
        return Math.abs(mTargetPosition - mIndexerMotor.getEncoder().getPosition()) > Constants.Indexer.Spinner.kPositionTolerance;
    }

    public void addBalls(int i)
    {
        mBallsHeld = Math.min(5, Math.max(0, mBallsHeld+i));
    }

    public void setBalls(int i)
    {
        mBallsHeld = Math.min(5, Math.max(0, i));
    }

    public int getBalls()
    {
        return mBallsHeld;
    }

    public boolean areFinsAligned()
    {
        double positionMod90 = mIndexerMotor.getEncoder().getPosition() % 90;
        return (positionMod90 >= 85 || positionMod90 <= 5); // if in a safe space to load a ball
    }
}
