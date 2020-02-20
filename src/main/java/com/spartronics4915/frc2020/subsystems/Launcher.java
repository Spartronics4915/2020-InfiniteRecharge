package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.LauncherCommands;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsAnalogEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Launcher extends SpartronicsSubsystem
{
    private SpartronicsMotor mFlywheelMasterMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private SpartronicsMotor mTurretMotor;
    private final Servo mAngleAdjusterMasterServo;
    private final Servo mAngleAdjusterFollowerServo;
    private final SpartronicsEncoder mTurretEncoder;

    private InterpolatingTreeMap<InterpolatingDouble, LauncherState> table;

    private double mTargetRPS;
    private Rotation2d mTargetAngle;

    private SimpleMotorFeedforward mFeedforwardCalculator;
    private final PIDController mTurretPIDController;
    private boolean mTurretZeroed;
    private Rotation2d mTargetTurretDirection;

    private static class LauncherState implements Interpolable<LauncherState>
    {
        public final Rotation2d hoodAngle;
        public final InterpolatingDouble flywheelSpeedRPS;

        public LauncherState(Rotation2d hoodAngle, InterpolatingDouble speedRPS)
        {
            this.hoodAngle = hoodAngle;
            this.flywheelSpeedRPS = speedRPS;
        }

        @Override
        public LauncherState interpolate(LauncherState endValue, double x)
        {
            return new LauncherState(hoodAngle.interpolate(endValue.hoodAngle, x),
                flywheelSpeedRPS.interpolate(endValue.flywheelSpeedRPS, x));
        }
    }

    public Launcher()
    {
        // ONE NEO for flywheel
        mFlywheelMasterMotor = SpartronicsMax.makeMotor(Constants.Launcher.kFlywheelMasterId);
        if (mFlywheelMasterMotor.hadStartupError())
        {
            mFlywheelMasterMotor = new SpartronicsSimulatedMotor(
                Constants.Launcher.kFlywheelMasterId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        mFlywheelMasterMotor.setVelocityGains(Constants.Launcher.kP, 0, 0, 0); // ref value is
                                                                               // 0.00036
        mFeedforwardCalculator = new SimpleMotorFeedforward(Constants.Launcher.kS,
            Constants.Launcher.kV, Constants.Launcher.kA);
        mFlywheelMasterMotor.setOutputInverted(false);
        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

        mTargetAngle = new Rotation2d();
        mTargetTurretDirection = new Rotation2d();

        // One BAG motor for turret
        mTurretMotor = SpartronicsSRX.makeMotor(Constants.Launcher.kTurretId,
            SensorModel.fromMultiplier(Math.toDegrees(1.0 / 1024.0 / 11.75 / 20.0) * 2.0));// UNITS ARE GOOD

        if (mTurretMotor.hadStartupError())
        {
            mTurretMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kTurretId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        mTurretMotor.setSoftLimits(45, -45);
        mTurretEncoder = mTurretMotor.getEncoder();
        mTurretPIDController = new PIDController(Constants.Launcher.kTurretP, 0,
            Constants.Launcher.kTurretD);
        mTurretPIDController.setTolerance(1.0);

        // Two Servos for angle adjustement
        mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
        mAngleAdjusterFollowerServo = new Servo(Constants.Launcher.kAngleAdjusterFollowerId);

        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

        setUpLookupTable(Constants.Launcher.kLookupTableSize, Constants.Launcher.kDistanceTable,
            Constants.Launcher.kAngleTable, Constants.Launcher.kRPSTable);
        mTurretZeroed = false;
        reset();
    }

    /**
     * Sets target RPS for flywheel to given RPS.
     * Call this in execute() method of a command to have the motor
     * constantly run at the target RPS.
     * <p>
     * Does not allow values greater than 90 RPS (currently, refer to
     * Constants.Launcher.kMaxRPS).
     * @param rps RPS you want the flywheel to target
     */
    public void runFlywheel(double rps)
    {
        mTargetRPS = Math.min(rps, Constants.Launcher.kMaxRPS);
        mFlywheelMasterMotor.setVelocity(mTargetRPS,
            mFeedforwardCalculator.calculate(mTargetRPS / 60.0));
    }

    /**
     * Raises the hood to the angle.
     * @param angle A Rotation2d for how far above horizontal you want the angle adjuster to go.
     */
    public void adjustHood(Rotation2d angle)
    {
        mTargetAngle = Rotation2d
            .fromDegrees(Math.min(angle.getDegrees(), Constants.Launcher.kMaxAngle.getDegrees()));
        mAngleAdjusterMasterServo.setAngle(mTargetAngle.getDegrees());
        mAngleAdjusterFollowerServo.setAngle(180 - mTargetAngle.getDegrees());
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Rotation2d expressing directed turret direction
     */
    public void turnTurret(Rotation2d absoluteAngle)
    {
        mTargetTurretDirection = absoluteAngle;
        double output = mTurretPIDController.calculate(mTurretEncoder.getPosition(),
            Util.limit(absoluteAngle.getDegrees(), Constants.Launcher.kMaxAngle.getDegrees()));
        mTurretMotor.setDutyCycle(output);
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Angle in degrees you want to turn the turret relative to the home position
     */
    public void turnTurret(double angle)
    {
        this.turnTurret(Rotation2d.fromDegrees(angle));
    }

    /**
     * Returns the current angle the turret is facing relative to straight ahead/home position
     * @return Current angle in degrees the turret is facing relative to the home position (forwards)
     */
    public Rotation2d getTurretDirection()
    {
        return Rotation2d.fromDegrees(mTurretEncoder.getPosition());
    }

    public Rotation2d getTargetTurretDirection()
    {
        return mTargetTurretDirection;
    }

    /**
     * Returns current target RPS of shooter
     * @return RPS that the flywheel is targeting
     */
    public double getTargetRPS()
    {
        return mTargetRPS;
    }

    /**
     * Returns current angle of angle adjuster
     * @return Current angle in degrees above horizontal of the angle adjuster
     */
    public Rotation2d getCurrentPitch()
    {
        // NEED ENC OR POT
        return Rotation2d.fromDegrees(mAngleAdjusterMasterServo.getPosition());
    }

    /**
     * Returns current RPS of shooter
     * @return The current RPS of the flywheel
     */
    public double getCurrentRPS()
    {
        return mFlywheelEncoder.getVelocity();
    }

    public Boolean isFlywheelSpun()
    {
        return getTargetRPS() * 0.95 <= getCurrentRPS();
    }

    /**
     * Computes and returns angle for angle adjuster based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return The angle in degrees above horizontal that is calculated to be necessary
     * to hit the target based off of the input distance
     */
    public Rotation2d calcPitch(double distance)
    {
        return table.getInterpolated(new InterpolatingDouble(distance)).hoodAngle;
    }

    /**
     * Computes and returns RPS based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPS calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPS(double distance)
    {
        return table.getInterpolated(new InterpolatingDouble(distance)).flywheelSpeedRPS.value;
    }

    /**
     * Returns whether or not the target is within the range and FOV of the turret
     * @return True if the target can be shot to
     */
    public boolean inRange()
    {
        // TODO figure out actual bounds of the range and make a check for the turret
        // rotation
        boolean inRange = true;
        return inRange;
    }

    /**
     * Returns whether or not the turret is at the intended velocity, angle, and rotation
     * @return true if the turret is ready
     */
    public boolean atTarget()
    {
        return (Math.abs(getTargetRPS() - getCurrentRPS()) < Constants.Launcher.kFlywheelVelocityTolerance)
            && (mTurretPIDController.atSetpoint());
    }

    /**
     * Resets shooter and stops flywheel
     */
    public void reset()
    {
        runFlywheel(0);
        adjustHood(Rotation2d.fromDegrees(0));
        //turnTurret(Rotation2d.fromDegrees(0));
    }

    public void setUpLookupTable(int size, double[] distances, double[] angles, double[] rps)
    {
        table = new InterpolatingTreeMap<>();
        for (int k = 0; k < size; k++)
        {
            table.put(new InterpolatingDouble(distances[k]), new LauncherState(
                Rotation2d.fromDegrees(angles[k]), new InterpolatingDouble(rps[k])));
        }
    }

    public void zeroTurret()
    {
        /*if (mTurretMotor.getOutputCurrent() > Constants.Launcher.kTurretStallAmps)
        {
            mTurretEncoder.setPosition(45.0);
            mTurretMotor.setDutyCycle(0.0);
            zeroed = true;
        }
        else if (!zeroed)
        {
            mTurretMotor.setDutyCycle(0.1);
        }*/
        mTurretEncoder.setPosition(0.0);
    }

    public boolean isZeroed()
    {
        return mTurretZeroed;
    }

    public void stopTurret()
    {
        mTurretMotor.setDutyCycle(0);
    }

    @Override
    public void periodic()
    {
        dashboardPutNumber("turretAngle", getTurretDirection().getDegrees());
        dashboardPutNumber("turretAngleTarget", getTargetTurretDirection().getDegrees());

        dashboardPutNumber("hoodAngle", getCurrentPitch().getDegrees());

        dashboardPutNumber("flywheelRPS", getCurrentRPS());
        dashboardPutNumber("flywheelRPSTarget", mTargetRPS);
    }
}
