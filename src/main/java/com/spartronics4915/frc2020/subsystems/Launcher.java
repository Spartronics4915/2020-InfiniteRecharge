package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * FIXME: all the calc methods need to work properly with a Rotation2d
 */
public class Launcher extends SpartronicsSubsystem
{
    private SpartronicsMotor mFlywheelMasterMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private SpartronicsMotor mTurretMotor;
    private AnalogPotentiometer mTurretPotentiometer;
    private Servo mAngleAdjusterMasterServo;
    private Servo mAngleAdjusterFollowerServo;

    private InterpolatingTreeMap<InterpolatingDouble, LauncherState> mLookupTable;

    private double targetRPS;
    private double targetAngle;
    private double targetRotation;

    private SimpleMotorFeedforward mFeedforwardCalculator;

    private static class LauncherState implements Interpolable<LauncherState>
    {
        public final Rotation2d hoodAngle;
        public final InterpolatingDouble flywheelSpeedRPS;
    }

    public Launcher()
    {
        mFlywheelMasterMotor = SpartronicsMax.makeMotor(Constants.Launcher.kFlywheelMasterId);
        if (mFlywheelMasterMotor.hadStartupError())
        {
            mFlywheelMasterMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kFlywheelMasterId);
            mTurretMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kTurretId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        mFlywheelMasterMotor.setVelocityGains(0.000389, 0, 0, 0); // TODO: Constants???
        mFlywheelMasterMotor.setOutputInverted(true);
        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();
        mFeedforwardCalculator = new SimpleMotorFeedforward(
            Constants.Launcher.kS, Constants.Launcher.kV, Constants.Launcher.kA);

        mTurretMotor = SpartronicsMax.makeMotor(Constants.Launcher.kTurretId, SensorModel.toRadians(360));
        if (mTurretMotor.hadStartupError())
        {
            mTurretMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kTurretId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        mTurretPotentiometer = new AnalogPotentiometer(Constants.Launcher.kTurretPotentiometerId, 360, -180); // TODO: Constants???

        mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
        mAngleAdjusterFollowerServo = new Servo(Constants.Launcher.kAngleAdjusterFollowerId);

        setUpLookupTable(Constants.Launcher.LookupTableSize, Constants.Launcher.DistanceTable,
            Constants.Launcher.AngleTable, Constants.Launcher.RPSTable);

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
        if (rps > Constants.Launcher.kMaxRPS)
            targetRPS = Constants.Launcher.kMaxRPS;
        else
            targetRPS = rps;
        mFlywheelMasterMotor.setVelocity(targetRPS, mFeedforwardCalculator.calculate(targetRPS));
        dashboardPutNumber("targetRPS", targetRPS);
    }

    /**
     * Raises the hood to the targetAngle.
     * @param angle Angle in degrees above horizontal you want the angle adjuster to go to.
     */
    public void adjustHood(double angle)
    {
        if (angle > Constants.Launcher.kMaxAngle)
            targetAngle = Constants.Launcher.kMaxAngle;
        else
            targetAngle = angle;
        mAngleAdjusterMasterServo.setAngle(targetAngle);
        mAngleAdjusterFollowerServo.setAngle(targetAngle);
        dashboardPutNumber("targetAngle", targetAngle);
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Angle in degrees you want to turn the turret relative to the home position
     */
    public void rotateTurret(double absoluteAngle)
    {
        // FIXME
        // need enc or pot
        // check if exceeds limits
        mTurretMotor.setPosition(absoluteAngle);
        dashboardPutNumber("targetRotation", targetRotation);
    }

    /**
     * Returns current target RPS of shooter
     * @return RPS that the flywheel is targeting
     */
    public double getTargetRPS()
    {
        return targetRPS;
    }

    /**
     * Returns current RPS of shooter
     * @return The current RPS of the flywheel
     */
    public double getCurrentRPS()
    {
        return mFlywheelEncoder.getVelocity();
    }

    /**
     * Returns target angle of angle adjuster
     * @return Angle in degrees above horizontal that the angle adjuster is targeting
     */
    public double getTargetAngle()
    {
        return targetAngle;
    }

    /**
     * Returns current angle of angle adjuster
     * @return Current angle in degrees above horizontal of the angle adjuster
     */
    public double getCurrentAngle()
    {
        // NEED ENC OR POT
        return mAngleAdjusterMasterServo.getPosition();
    }

    /**
     * Returns the target angle the turret is facing relative to straight ahead / home position
     * @return Target angle in degrees the turret is facing relative to the home position (forwards)
     */
    public double getTargetRotation()
    {
        return targetRotation;
    }

    /**
     * Returns the current angle the turret is facing relative to straight ahead/home position
     * @return Current angle in degrees the turret is facing relative to the home position (forwards)
     */
    public double getCurrentRotation()
    {
        return mTurretPotentiometer.get();
    }

    /**
     * Returns whether or not the target is within the range that the turret can rotate to
     * @return True if the target is within the turret's range of rotation, else false
     */
    public boolean inFOV()
    {
        // FIXME lol
        boolean inRotationRange = true;
        return inRotationRange;
    }

    /**
     * Returns whether or not the target is within the horizontal distance from the target
     * that the shooter can shoot, used by driver
     * @return True if the target is within the range the shooter is capable of hitting, else false
     */
    public boolean inRange()
    {
        // FIXME
        boolean inRange = true;
        return inRange;
    }

    /**
     * Computes and returns RPS based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPS calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPS(double distance)
    {
        return mLookupTable.getInterpolated(
            new InterpolatingDouble(distance)).flywheelSpeedRPS.value;
    }

    /**
     * Computes and returns angle for angle adjuster based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return The angle in degrees above horizontal that is calculated to be necessary
     * to hit the target based off of the input distance
     */
    public Rotation2d calcPitch(double distance)
    {
        Rotation2d angle = mLookupTable.getInterpolated(
            new InterpolatingDouble(distance)).hoodAngle;
        return angle;
    }

    // FIXME
    public double calcRotation(Rotation2d rotation)
    {
        return -1;
    }

    public void setUpLookupTable(int size, double[] distances, double[] angles, double[] rps)
    {
        mLookupTable = new InterpolatingTreeMap<>();
        mLookupTable.put(new InterpolatingDouble(0.1),
            new LauncherState(Rotation2d.fromDegrees(45.0), new InterpolatingDouble(90.0)));
        for (int k = 0; k < size; k++)
        {
            mLookupTable.put(new InterpolatingDouble(distances[k]), new LauncherState(
                Rotation2d.fromDegrees(angles[k]), new InterpolatingDouble(rps[k])));
        }
    }

    /**
     * Resets shooter and stops flywheel
     */
    public void reset()
    {
        runFlywheel(0);
        adjustHood(0);
        rotateTurret(0);
    }
}
