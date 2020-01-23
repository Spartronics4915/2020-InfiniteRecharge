package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX.SpartronicsSRXEncoder;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SpartronicsSubsystem {
    private SpartronicsMax mFlywheelMasterMotor;
    private SpartronicsMax mFlywheelFollowerMotor;
    private SpartronicsEncoder mFlywheelEncoder;
    private SpartronicsSRX mAngleAdjusterMotor;
    private SpartronicsEncoder mAngleAdjusterEncoder;
    private SpartronicsSRX mTurretMotor;
    private SpartronicsEncoder mTurretEncoder;
    private double targetRPM;
    private double targetAngle;

    public Launcher() {
        boolean success = false;
        try {
            // Two NEOs for flywheel (Master and follower, opposite directions)
            mFlywheelMasterMotor = new SpartronicsMax(Constants.Launcher.kFlywheelMasterID,null);
            mFlywheelFollowerMotor = new SpartronicsMax(Constants.Launcher.kFlywheelFollowerID,null);
            mFlywheelFollowerMotor.follow(mFlywheelMasterMotor);
            mFlywheelFollowerMotor.setOutputInverted(true);
            mFlywheelEncoder = mFlywheelMasterMotor.getEncoder();

            // One snowblower for angle adjustement
            mAngleAdjusterMotor = new SpartronicsSRX(Constants.Launcher.kAngleAdjusterID,null);
            mAngleAdjusterEncoder = mAngleAdjusterMotor.getEncoder();

            // One BAG motor for turret
            mTurretMotor = new SpartronicsSRX(Constants.Launcher.kTurretID,null);
            mTurretEncoder = mTurretMotor.getEncoder();

            success = true;
        } 
        catch (Exception e) {
            success = false;
            logException("Could not instantiate Launcher: ", e);
        }
        logInitialized(success);
    }

    public void runFlywheel() {
        mFlywheelMasterMotor.setVelocity(targetRPM);
    }

    /**
     * @param relativeAngle Angle in degrees you want to turn the turret relative to the current angle
     */
    public void turnTurret(double relativeAngle) {
        //rotates turret a specific angle relative to its current angle
    }

    /**
     * @return Current angle in degrees the turret is facing relative to the home position (forwards)
     */
    public double getTurretDirection() {
        //returns the current angle the turret is facing relative to straight ahead/home position
        return mTurretEncoder.getPosition();
    }

    /**
     * @param angle Angle in degrees above horizontal you want the angle adjuster to go to
     */
    public void setPitch(double angle) {
        //sets target angle to given angle
        targetAngle = angle;
    }

    /**
     * @param rpm RPM you want the flywheel to target
     */
    public void setRPM(double rpm) {
        //sets target rpm for flywheel to given rpm
        targetRPM = rpm;
    }

    /**
     * @return Angle in degrees above horizontal that the angle adjuster is targeting
     */
    public double getTargetPitch() {
        //returns current target angle of angle adjuster
        return targetAngle;
    }

    /**
     * @return RPM that the flywheel is targeting
     */
    public double getTargetRPM() {
        //returns current target RPM of shooter
        return targetRPM;
    }

    /**
     * @return Current angle in degrees above horizontal of the angle adjuster
     */
    public double getCurrentPitch() {
        //returns current angle of angle adjuster
        //NEED ENC OR POT
        mAngleAdjusterEncoder.getPosition();
        return 0.0;
    }

    /**
     * @return The current RPM of the flywheel
     */
    public double getCurrentRPM() {
        //returns current RPM of shooter
        return mFlywheelEncoder.getVelocity();
    }

    /**
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return The angle in degrees above horizontal that is calculated to be necessary to hit the target based off of the input distance
     */
    public double calcPitch(double distance) {
        //computes and returns angle for angle adjuster based on input distance
        double angle = 0.0;
        return angle;
    }

    /**
     * @param distance Horizontal distance in meters from the shooter to the target
     * @return RPM calculated to be necessary to hit the target based of of the input distance
     */
    public double calcRPM(double distance) {
        //computes and returns RPM based on input distance
        double RPM = 0.0;
        return RPM;
    }

    /**
     * @return True if the target is within the turret's range of rotation, else false
     */
    public boolean inFOV() {
        //returns whether or not the target is within the range that the turret can rotate to, used by driver
        boolean inRotationRange = true;
        return inRotationRange;
    }

    /**
     * @return True if the target is within the horizontal distance from the target the shooter is capable of shooting to, else false
     */
    public boolean inRange() {
        //returns whether or not the target is within the range that the shooter can shoot, used by driver
        boolean inRange = true;
        return inRange;
    }

    public void reset() {
        //reset
    }
    
    public void stop() { // Unlikely to be used
        // stop all motors, including the flywheel
    }
}
