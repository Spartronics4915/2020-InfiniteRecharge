package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsAnalogEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
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
    private SpartronicsAnalogEncoder mTurretEncoder;

    private InterpolatingTreeMap<InterpolatingDouble, LauncherState> table;

    private double targetRPS;
    private Rotation2d targetAngle;

    private SimpleMotorFeedforward mFeedforwardCalculator;
    private final PIDController mTurretPIDController;

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
            mFlywheelMasterMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kFlywheelMasterId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        mFlywheelMasterMotor.setVelocityGains(Constants.Launcher.kP, 0, 0, 0); //ref value is 0.00036
        mFeedforwardCalculator = new SimpleMotorFeedforward(Constants.Launcher.kS,
            Constants.Launcher.kV, Constants.Launcher.kA);
        mFlywheelMasterMotor.setOutputInverted(true);
        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

        // One BAG motor for turret
        mTurretMotor = SpartronicsSRX.makeMotor(Constants.Launcher.kTurretId,
            SensorModel.toRadians(360));

        if (mTurretMotor.hadStartupError())
        {
            mTurretMotor = new SpartronicsSimulatedMotor(Constants.Launcher.kTurretId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }

        var analogInput = new AnalogInput(Constants.Launcher.kTurretPotentiometerId);
        analogInput.setAverageBits(4);
        mTurretEncoder = new SpartronicsAnalogEncoder(analogInput);
        mTurretEncoder.setDistancePerRotation(1);
        mTurretPIDController = new PIDController(Constants.Launcher.kTurretP, 0, Constants.Launcher.kTurretD);

        // Two Servos for angle adjustement
        mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
        mAngleAdjusterFollowerServo = new Servo(Constants.Launcher.kAngleAdjusterFollowerId);

        mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

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
        targetRPS = Math.min(rps, Constants.Launcher.kMaxRPS);
        mFlywheelMasterMotor.setVelocity(targetRPS, mFeedforwardCalculator.calculate(targetRPS / 60.0));
    }

    /**
     * Raises the hood to the angle.
     * @param angle A Rotation2d for how far above horizontal you want the angle adjuster to go.
     */
    public void adjustHood(Rotation2d angle)
    {
        targetAngle = Rotation2d.fromDegrees(Math.min(angle.getDegrees(),
            Constants.Launcher.kMaxAngle.getDegrees()));
        mAngleAdjusterMasterServo.setAngle(targetAngle.getDegrees());
        mAngleAdjusterFollowerServo.setAngle(180 - targetAngle.getDegrees());
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Angle in degrees you want to turn the turret relative to the home position
     */
    public void turnTurret(Rotation2d absoluteAngle)
    {
        // FIXME: The turret should expose methods similar to getTarget/CurrentPitch
        double output = mTurretPIDController.calculate(mTurretEncoder.get(), absoluteAngle.getDegrees());
        mTurretMotor.setDutyCycle(output);
    }

    /**
     * Returns the current angle the turret is facing relative to straight ahead/home position
     * @return Current angle in degrees the turret is facing relative to the home position (forwards)
     */
    public Rotation2d getTurretDirection()
    {
        return Rotation2d.fromDegrees(mTurretEncoder.get());
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
    public Rotation2d getCurrentPitch() // TODO: Verify degrees vs. radians
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
        return table.getInterpolated(
            new InterpolatingDouble(distance)).flywheelSpeedRPS.value;
    }

    /**
     * Returns whether or not the target is within the range and FOV of the turret
     * @return True if the target can be shot to
     */
    public boolean inRange()
    {
        // FIXME
        boolean inRange = true;
        return inRange;
    }

    /**
     * Returns whether or not the turret is at the intended velocity, angle, and rotation
     * @return true if the turret is ready
     */
    public boolean atTarget()
    {
        // FIXME: Include the turret rotation
        return (getTargetRPS() == getCurrentRPS())
            && (getTargetPitch() == getCurrentPitch());
    }

    /**
     * Resets shooter and stops flywheel
     */
    public void reset()
    {
        runFlywheel(0);
        adjustHood(Rotation2d.fromDegrees(0));
        turnTurret(Rotation2d.fromDegrees(0));
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

    @Override
    public void periodic()
    {
        dashboardPutNumber("turretAngle", getTurretDirection().getDegrees());
        dashboardPutNumber("currentFlywheelRPS", getCurrentRPS());
        dashboardPutNumber("currentHoodAngle", getCurrentPitch().getDegrees());
    }
}
