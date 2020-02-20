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
    private SpartronicsMotor mKickerMotor;
    private SpartronicsMotor mTransferMotor;

    private SensorModel mIndexerModel;
    private SensorModel mKickerModel;
    private SensorModel mTransferModel;

    private DigitalInput mLimitSwitch;
    private DigitalInput mOpticalProxSensor;
    private DigitalInput mIntakeProxSensor;

    private boolean mIsLaunching = false;
    private boolean mIsTransferring = false;
    private boolean mHasZeroed = false;

    private int mBallsHeld = 0;

    public boolean mIsFull = false;

    public Indexer()
    {
        // Set up Spinner
        mIndexerModel = SensorModel.fromMultiplier(Constants.Indexer.Spinner.kConversionRatio);
        mIndexerMotor = SpartronicsMax.makeMotor(Constants.Indexer.Spinner.kMotorId, mIndexerModel);
        // Set up Loader
        mKickerModel = SensorModel.fromMultiplier(Constants.Indexer.Loader.kConversionRatio);
        mKickerMotor = SpartronicsSRX.makeMotor(Constants.Indexer.Loader.kMotorId, mKickerModel); // BAG motor
        // Set up Transfer
        mTransferModel = SensorModel.fromMultiplier(Constants.Indexer.Transfer.kConversionRatio);
        mTransferMotor = SpartronicsMax.makeMotor(Constants.Indexer.Transfer.kMotorId, mTransferModel);

        if (mIndexerMotor.hadStartupError() || mKickerMotor.hadStartupError() || mTransferMotor.hadStartupError())
        {
            mIndexerMotor = new SpartronicsSimulatedMotor(Constants.Indexer.Spinner.kMotorId);
            mKickerMotor = new SpartronicsSimulatedMotor(Constants.Indexer.Loader.kMotorId);
            mTransferMotor = new SpartronicsSimulatedMotor(Constants.Indexer.Transfer.kMotorId);
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
        mIndexerMotor.setBrakeMode(true);

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
        return !mLimitSwitch.get();
    }

    /**
     * Sets the spinner to a specific velocity
     * @param dutyCycle the velocity to spin the spinner at
     */
    public void spinAt(double dutyCycle)
    {
        mIndexerMotor.setPercentOutput(dutyCycle);
    }

    /**
     * Sets the spinner encoder to zero at it's current position
     */
    public void setZero()
    {
        mHasZeroed = true;
        mIndexerMotor.getEncoder().setPosition(0);
    }

    public void unzero()
    {
        mHasZeroed = false;
    }

    public boolean hasZeroed()
    {
        return mHasZeroed;
    }

    /**
     * @return whether or not a ball is loaded in the first slot
     */
    public boolean getSlotBallLoaded()
    {
        return !mOpticalProxSensor.get();
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
        }
    }

    /**
     * Returns spinner to "0" position on the encoder
     */
    public void returnToHome()
    {
        mTargetPosition = 0;
    }

    /**
     * Move spinner to nearest position
     */
    public void toNearestQuarterRotation()
    {
        // Rotates to nearest quarter rotation
        mTargetPosition = Math.ceil(mIndexerMotor.getEncoder().getPosition() * 4) / 4;
    }

    /**
     * Runner spinner motor
     */
    public void goToPosition()
    {
        if (isJamming())
            mIndexerMotor.setPercentOutput(-0.3);
        else
            mIndexerMotor.setPosition(mTargetPosition);
    }

    /**
     * Start loading balls into the shooter
     */
    public void launch()
    {
        mIsLaunching = true;
        mKickerMotor.setPercentOutput(Constants.Indexer.Loader.kSpeed);
    }

    /**
     * Stop loading balls into the shooter
     */
    public void endLaunch()
    {
        mIsLaunching = false;
        mKickerMotor.setPercentOutput(0);
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
        mTransferMotor.setPercentOutput(Constants.Indexer.Transfer.kSpeed);
    }

    public void endTransfer()
    {
        mIsTransferring = false;
        mTransferMotor.setPercentOutput(0);
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
        mTransferMotor.setNeutral();
        mKickerMotor.setNeutral();
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
        return Math.abs(mTargetPosition - mIndexerMotor.getEncoder().getPosition()) * 360 < Constants.Indexer.Spinner.kPositionTolerance;
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
        double positionMod90 = (mIndexerMotor.getEncoder().getPosition() * 360) % 90;
        return (positionMod90 >= (90 - Constants.Indexer.Spinner.kPositionTolerance) || positionMod90 <= Constants.Indexer.Spinner.kPositionTolerance); // if in a safe space to load a ball
    }

    public boolean isJamming()
    {
        return mIndexerMotor.getOutputCurrent() >= Constants.Indexer.Spinner.kStallThreshold;
    }

    @Override
    public void periodic()
    {
        dashboardPutNumber("targetPosition", mTargetPosition);
        dashboardPutNumber("position", mIndexerMotor.getEncoder().getPosition());
        dashboardPutNumber("ballsHeld", mBallsHeld);
        dashboardPutBoolean("isLaunching", mIsLaunching);
        dashboardPutBoolean("isTransferring", mIsTransferring);
    }
}
