package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.LauncherCommands;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Launcher extends SpartronicsSubsystem
{
    private SpartronicsMotor mFlywheelMasterMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private Servo mAngleAdjusterMasterServo;
    private Servo mAngleAdjusterFollowerServo;
    private SpartronicsMotor mTurretMotor;
    private AnalogPotentiometer mTurretPotentiometer;

    private double targetRPS;
    private double targetAngle;

    private SimpleMotorFeedforward mFeedforwardCalculator;

    public Launcher()
    {
        // Construct your hardware here
        boolean success = false;
        try
        {
            // ONE NEO for flywheel
            mFlywheelMasterMotor = SpartronicsMax.makeMotor(
                /*Constants.Launcher.kFlywheelMasterID*/2,
                SensorModel.fromMultiplier(1)/*, Constants.Launcher.kFlywheelFollowerID*/);
                mFlywheelMasterMotor.setOutputInverted(true);
            mFlywheelMasterMotor.setVelocityGains(0.000389, 0, 0, 0);
            mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

            // Two Servos for angle adjustement
            mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
            mAngleAdjusterFollowerServo = new Servo(Constants.Launcher.kAngleAdjusterFollowerId);
            // One NEO 550 motor for turret
            mTurretMotor = SpartronicsMax.makeMotor(Constants.Launcher.kTurretId,
                    SensorModel.toRadians(360));
            turnTurret(0);
            mTurretPotentiometer = new AnalogPotentiometer(
                Constants.Launcher.kTurretPotentiometerId, 90, -45);
            
            mFeedforwardCalculator = new SimpleMotorFeedforward(Constants.Launcher.kS, 
                Constants.Launcher.kV, Constants.Launcher.kA);
            success = true;
        }
        catch (Exception e)
        {
            // TODO: handle exception
            logException("Could not instantiate Launcher: ", e);
            success = false;
        }
        logInitialized(success);
    }

    /**
     * call this in execute() method of a command to have the motor constantly run at the target rpm
     */
    public void runFlywheel()
    {
        mFlywheelMasterMotor.setVelocity(targetRPS, mFeedforwardCalculator.calculate(targetRPS));
        System.out.println("Flywheel's current rps is " + getCurrentRPS());
    }

    /**
     * Rotates turret to a specific angle relative to the home position
     * @param absoluteAngle Angle in degrees you want to turn the turret relative to the home position
     */
    public void turnTurret(double absoluteAngle)
    {
        // need enc or pot\
        mTurretMotor.setPosition(absoluteAngle);
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
        if (angle > 30)
        {
            angle = 30;
        }
        targetAngle = angle;
    }

    /**
     * Sets target rpm for flywheel to given rpm
     * @param rpm RPM you want the flywheel to target
     */
    public void setRPS(double rps)
    {
        targetRPS = rps;
    }

    /**
     * Returns current target angle of angle adjuster
     * @return Angle in degrees above horizontal that the angle adjuster is targeting
     */
    public double getTargetPitch()
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
    public double calcPitch(double distance)
    {
        double angle = 0.0;
        return angle;
    }

    /**
     * Computes and returns RPS based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPS calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPS(double distance)
    {
        double RPS = 0.0;
        return RPS;
    }

    /**
     * Returns whether or not the target is within the range that the turret can rotate to, used by driver
     * @return True if the target is within the turret's range of rotation, else false
     */
    public boolean inFOV()
    {
        boolean inRotationRange = true;
        return inRotationRange;
    }

    /**
     * Returns whether or not the target is within the range that the shooter can shoot, used by driver
     * @return True if the target is within the horizontal distance from the target the shooter is capable of shooting to, else false
     */
    public boolean inRange()
    {
        boolean inRange = true;
        return inRange;
    }

    /**
     * Reverses flywheel motors
     */
    public void reverse()
    {

    }

    /**
     * Resets shooter
     */
    public void reset()
    {
        /*setRPS(0);
        mFlywheelMasterMotor.setBrakeMode(true);
        setPitch(0);
        turnTurret(0);*/
    }
    // The exception to this is a general-functionality stop() method.
}
