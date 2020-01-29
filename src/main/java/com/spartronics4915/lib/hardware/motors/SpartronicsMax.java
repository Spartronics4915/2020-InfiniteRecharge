package com.spartronics4915.lib.hardware.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SpartronicsMax implements SpartronicsMotor
{

    private static final int kVelocitySlotIdx = 0;
    private static final int kPositionSlotIdx = 1;

    private final double kRPMtoRPS = 1 / 60;

    private CANSparkMax mSparkMax;
    private SpartronicsEncoder mEncoder;
    private SensorModel mSensorModel;

    private boolean mBrakeMode = false;
    /** Volts */
    private double mVoltageCompSaturation = 12.0;
    /** Native units/sec, converted to meters on get and set */
    private double mMotionProfileCruiseVelocity = 0.0;
    /** Native units/sec^2, converted to meters on get and set */
    private double mMotionProfileAcceleration = 0.0;
    private boolean mUseMotionProfileForPosition = false;

    public class SpartronicsMaxEncoder implements SpartronicsEncoder
    {

        @Override
        public double getVelocity()
        {
            return mSensorModel.toCustomUnits(mSparkMax.getEncoder().getVelocity());
        }

        @Override
        public double getPosition()
        {
            return mSensorModel.toCustomUnits(mSparkMax.getEncoder().getPosition());
        }

        @Override
        public void setPhase(boolean isReversed)
        {
            mSparkMax.setInverted(isReversed);
        }

        @Override
        public void setPosition(double position) {
            mSparkMax.getEncoder().setPosition(position);
        }

        
    }

    public SpartronicsMax(int deviceNumber, SensorModel sensorModel)
    {
        this(new CANSparkMax(deviceNumber, MotorType.kBrushless), sensorModel);
    }

    public SpartronicsMax(CANSparkMax spark, SensorModel sensorModel)
    {
        mSparkMax = spark;
        mSensorModel = sensorModel;
        mEncoder = new SpartronicsMaxEncoder();
        mSparkMax.getEncoder().setPosition(0);
        mSparkMax.getEncoder().setVelocityConversionFactor(kRPMtoRPS); // Set conversion factor.

        // mSparkMax.configFactoryDefault();
        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public SpartronicsEncoder getEncoder()
    {
        return mEncoder;
    }

    @Override
    public double getVoltageOutput()
    {
        return mSparkMax.getBusVoltage() * mSparkMax.getAppliedOutput();
    }

    @Override
    public boolean getOutputInverted()
    {
        return mSparkMax.getInverted();
    }

    @Override
    public void setOutputInverted(boolean inverted)
    {
        mSparkMax.setInverted(inverted);
    }

    @Override
    public boolean getBrakeMode()
    {
        return mBrakeMode;
    }

    @Override
    public void setBrakeMode(boolean mode)
    {
        mBrakeMode = mode;
        mSparkMax.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public double getVoltageCompSaturation()
    {
        return mVoltageCompSaturation;
    }

    @Override
    public void setVoltageCompSaturation(double voltage)
    {
        mVoltageCompSaturation = voltage;
        mSparkMax.enableVoltageCompensation(mVoltageCompSaturation);
    }

    @Override
    public double getMotionProfileCruiseVelocity()
    {
        return mSensorModel.toCustomUnits(mMotionProfileCruiseVelocity);
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond)
    { // Set to slot
        mMotionProfileCruiseVelocity = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mSparkMax.getPIDController().setSmartMotionMaxVelocity((int) mMotionProfileCruiseVelocity,
                kVelocitySlotIdx);
    }

    @Override
    public double getMotionProfileMaxAcceleration()
    {
        return mSensorModel.toCustomUnits(mMotionProfileAcceleration);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersPerSecondSq)
    {
        mMotionProfileAcceleration = mSensorModel.toNativeUnits(accelerationMetersPerSecondSq);
        mSparkMax.getPIDController().setSmartMotionMaxAccel((int) mMotionProfileAcceleration,
                kVelocitySlotIdx);
    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile)
    {
        mUseMotionProfileForPosition = useMotionProfile;
    }

    @Override
    public void setDutyCycle(double dutyCycle, double arbitraryFeedForwardVolts)
    {
        mSparkMax.getPIDController().setReference(dutyCycle, ControlType.kDutyCycle, 0,
                arbitraryFeedForwardVolts);
    }

    @Override
    public void setDutyCycle(double dutyCycle)
    {
        setDutyCycle(dutyCycle, 0.0);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts)
    {
        double velocityNative = mSensorModel.toNativeUnits(velocityMetersPerSecond);
        mSparkMax.getPIDController().setReference(velocityNative, ControlType.kVelocity,
                kVelocitySlotIdx, arbitraryFeedForwardVolts);
    }

    @Override
    public void setVelocityGains(double kP, double kD)
    {
        setVelocityGains(kP, 0, kD, 0);
    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF)
    {
        mSparkMax.getPIDController().setP(kP, kVelocitySlotIdx);
        mSparkMax.getPIDController().setI(kI, kVelocitySlotIdx);
        mSparkMax.getPIDController().setD(kD, kVelocitySlotIdx);
        mSparkMax.getPIDController().setFF(kF, kVelocitySlotIdx);
    }

    @Override
    public void setPosition(double positionMeters)
    {
        double positionNativeUnits = mSensorModel.toNativeUnits(positionMeters);
        mSparkMax.getPIDController().setReference(positionNativeUnits,
                mUseMotionProfileForPosition ? ControlType.kSmartMotion : ControlType.kPosition,
                kPositionSlotIdx);
    }

    @Override
    public void setPositionGains(double kP, double kD)
    {
        setPositionGains(kP, 0, kD, 0);
    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF)
    {
        mSparkMax.getPIDController().setP(kP, kPositionSlotIdx);
        mSparkMax.getPIDController().setI(kI, kPositionSlotIdx);
        mSparkMax.getPIDController().setD(kD, kPositionSlotIdx);
        mSparkMax.getPIDController().setFF(kF, kPositionSlotIdx);
    }

    public void follow(SpartronicsMax other)
    {
        mSparkMax.follow(other.mSparkMax);
    }

    @Override
    public SensorModel getSensorModel()
    {
        return mSensorModel;
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond)
    {
        setVelocity(velocityMetersPerSecond, 0.0);
    }

    @Override
    public void setNeutral()
    {
        mSparkMax.getPIDController().setReference(0.0, ControlType.kDutyCycle, 0);
    }

}
