package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.LauncherDefaultCommand;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;

public class Launcher extends SpartronicsSubsystem
{
    private SpartronicsMotor mFlywheelMasterMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private Servo mAngleAdjusterMasterServo;
    private Servo mAngleAdjusterFollowerServo;
    private SpartronicsMotor mTurretMotor;
    private AnalogPotentiometer mTurretPotentiometer;
    private double targetRPM;
    private double targetAngle;

    public Launcher()
    {
        // Construct your hardware here
        boolean success = false;
        try
        {
            // ONE NEO for flywheel
            mFlywheelMasterMotor = SpartronicsMax.makeMotor(Constants.Launcher.kFlywheelMasterId,
                SensorModel.toRadians(1), Constants.Launcher.kFlywheelFollowerId);// new
                                                                                  // SpartronicsMax(Constants.Launcher.kFlywheelMasterID,SensorModel.toRadians(1));
            // One NEO 550 motor for turret
            mTurretMotor = SpartronicsMax.makeMotor(Constants.Launcher.kTurretId,
                SensorModel.toRadians(360));
            if (mFlywheelMasterMotor.hadStartupError() || mTurretMotor.hadStartupError())
            {
                mFlywheelMasterMotor = new SpartronicsSimulatedMotor();
                mTurretMotor = new SpartronicsSimulatedMotor();
                logInitialized(false);
            }
            else
            {
                logInitialized(true);
            }
            mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

            // Two Servos for angle adjustement
            mAngleAdjusterMasterServo = new Servo(Constants.Launcher.kAngleAdjusterMasterId);
            mAngleAdjusterFollowerServo = new Servo(
                Constants.Launcher.kAngleAdjusterFollowerId);


            turnTurret(0);
            mTurretPotentiometer = new AnalogPotentiometer(
                Constants.Launcher.kTurretPotentiometerId, 90, -45);
            setDefaultCommand(new LauncherDefaultCommand(this));
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
        mFlywheelMasterMotor.setVelocity(targetRPM);
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
    public void setRPM(double rpm)
    {
        targetRPM = rpm;
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
     * Returns current target RPM of shooter
     * @return RPM that the flywheel is targeting
     */
    public double getTargetRPM()
    {
        return targetRPM;
    }

    /**
     * Returns current angle of angle adjuster
     * @return Current angle in degrees above horizontal of the angle adjuster
     */
    public double getCurrentPitch()
    {
        // NEED ENC OR POT
        double pitch = mAngleAdjusterMasterServo.getPosition();
        return pitch;
    }

    /**
     * Returns current RPM of shooter
     * @return The current RPM of the flywheel
     */
    public double getCurrentRPM()
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
     * Computes and returns RPM based on input distance
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPM calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPM(double distance)
    {
        double RPM = 0.0;
        return RPM;
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
        setRPM(0);
        mFlywheelMasterMotor.setBrakeMode(true);
        setPitch(0);
        turnTurret(0);
    }
    // The exception to this is a general-functionality stop() method.
}
