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

// TODO: Rework this whole "set then run" method thing
public class Launcher extends SpartronicsSubsystem
{
    private SpartronicsMotor mFlywheelMasterMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private SpartronicsMotor mTurretMotor;
    private AnalogPotentiometer mTurretPotentiometer;
    private Servo mAngleAdjusterMasterServo;
    private Servo mAngleAdjusterFollowerServo;

    private InterpolatingTreeMap<InterpolatingDouble, LauncherState> table;

    private double targetRPS;
    private Rotation2d targetAngle;

    private SimpleMotorFeedforward mFeedforwardCalculator;

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

        mFlywheelMasterMotor.setVelocityGains(0.000389, 0, 0, 0);
        mFeedforwardCalculator = new SimpleMotorFeedforward(Constants.Launcher.kS,
            Constants.Launcher.kV, Constants.Launcher.kA);
        mFlywheelMasterMotor.setOutputInverted(true);
        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

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

        mTurretPotentiometer = new AnalogPotentiometer(Constants.Launcher.kTurretPotentiometerId, 360, -180);

        mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
        mAngleAdjusterFollowerServo = new Servo(Constants.Launcher.kAngleAdjusterFollowerId);

        setUpLookupTable(Constants.Launcher.LookupTableSize, Constants.Launcher.DistanceTable,
            Constants.Launcher.AngleTable, Constants.Launcher.RPSTable);
        turnTurret(0);

        dashboardPutNumber("Flywheel RPS", 0);
    }

    /**
     * Call this in execute() method of a command to have the motor constantly run at the target RPM
     */
    public void runFlywheel()
    {
        mFlywheelMasterMotor.setVelocity(targetRPS, mFeedforwardCalculator.calculate(targetRPS));
    }

    /**
     * Raises the hood to the targetAngle
     */
    public void adjustHood()
    {
        mAngleAdjusterMasterServo.setAngle(targetAngle.getDegrees());
        mAngleAdjusterFollowerServo.setAngle(targetAngle.getDegrees());
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Angle in degrees you want to turn the turret relative to the home position
     */
    public void turnTurret(double absoluteAngle)
    {
        // need enc or pot
        // FIXME
        // mTurretMotor.setPosition(absoluteAngle);
    }

    /**
     * Returns the current angle the turret is facing relative to straight ahead/home position
     * @return Current angle in degrees the turret is facing relative to the home position (forwards)
     */
    public double getTurretDirection()
    {
        return mTurretPotentiometer.get();
    }

    /**
     * Sets target angle to given angle
     * @param angle Angle in degrees above horizontal you want the angle adjuster to go to
     */
    public void setPitch(double angle)
    {
        if (angle > 30.0)
            angle = 30.0;
        targetAngle = Rotation2d.fromDegrees(angle);
    }

    /**
     * Sets target rpm for flywheel to given RPS
     * <p>
     * Does not allow values greater than 90 (currently, refer to Constants.Launcher.kMaxRPS) RPS.
     *
     * @param rpm RPM you want the flywheel to target
     */
    public void setRPS(double rps)
    {
        if (rps > Constants.Launcher.kMaxRPS)
            targetRPS = Constants.Launcher.kMaxRPS;
        else
            targetRPS = rps;
    }

    /**
     * Returns current target angle of angle adjuster
     * @return Angle in degrees above horizontal that the angle adjuster is targeting
     */
    public Rotation2d getTargetPitch()
    {
        return targetAngle;
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
     * Returns current angle of angle adjuster
     * @return Current angle in degrees above horizontal of the angle adjuster
     */
    public double getCurrentPitch()
    {
        // NEED ENC OR POT
        return mAngleAdjusterMasterServo.getPosition();
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
     * Computes and returns angle for angle adjuster based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return The angle in degrees above horizontal that is calculated to be necessary to hit the target based off of the input distance
     */
    public Rotation2d calcPitch(double distance)
    {
        Rotation2d angle = table.getInterpolated(new InterpolatingDouble(distance)).hoodAngle;
        return angle;
    }

    /**
     * Computes and returns RPS based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPS calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPS(double distance)
    {
        double RPS = table
            .getInterpolated(new InterpolatingDouble(distance)).flywheelSpeedRPS.value;
        return RPS;
    }

    /**
     * Returns whether or not the target is within the range that the turret can rotate to, used by driver
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
        // FIXME lol
        boolean inRange = true;
        return inRange;
    }

    /**
     * Resets shooter and stops flywheel
     */
    public void reset()
    {
        setRPS(0);
        mFlywheelMasterMotor.setBrakeMode(true);
        setPitch(0);
        turnTurret(0);
    }

    public void setUpLookupTable(int size, double[] distances, double[] angles, double[] rps)
    {
        table = new InterpolatingTreeMap<>();
        table.put(new InterpolatingDouble(0.1),
            new LauncherState(Rotation2d.fromDegrees(45.0), new InterpolatingDouble(90.0)));
        for (int k = 0; k < size; k++)
        {
            table.put(new InterpolatingDouble(distances[k]), new LauncherState(
                Rotation2d.fromDegrees(angles[k]), new InterpolatingDouble(rps[k])));
        }
    }
}
