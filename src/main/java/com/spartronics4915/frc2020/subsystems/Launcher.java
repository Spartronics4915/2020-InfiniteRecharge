package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Launcher extends SpartronicsSubsystem {
    CANSparkMax m_flywheelMasterMotor;
    CANSparkMax m_flywheelFollowerMotor;
    TalonSRX m_angleadjusterMotor;
    TalonSRX m_turretMotor;
    public Launcher() {
        // Construct your hardware here
        logInitialized(true);
        //Two NEOs for flywheel (Master and follower, opposite directions)
        m_flywheelMasterMotor = new CANSparkMax(Constants.kFlywheelMasterID,MotorType.kBrushless);
        m_flywheelFollowerMotor = new CANSparkMax(Constants.kFlywheelFollowerID,MotorType.kBrushless);
        m_flywheelFollowerMotor.follow(m_flywheelMasterMotor, true);
        //One snowblower for angle adjustement
        m_angleadjusterMotor = new TalonSRX(Constants.kAngleAdjusterID);
        //One BAG motor for turret
        m_turretMotor = new TalonSRX(Constants.kTurretID);
    }

    // Outline your API here by creating specific methods.
    // Each method should perform a _singular action_
    // - eg. instead of a setIntake method, control each intake motor individually
    // setIntake functionality should be implemented in a command.
    public void turnToTarget() {
        //rotates turret to face target
    }

    public void setAngle(double angle) {
        //adjusts the angle to given angle
    }

    public void setRPM(double rpm) {
        //begins spinning the flywheel at the given RPM
    }

    public double getAngle(double distance) {
        //computes and returns angle based on input distance
        double angle=0.0;
        return angle;
    }

    public double getRPM(double distance) {
        //computes and returns RPM based on input distance
        double RPM=0.0;
        return RPM;
    }

    public boolean isBallShot() {
        //returns whether or not a ball has been shot
        boolean isShot=true;
        return isShot;
    }
    public boolean inRotation() {
        //returns whether or not the target is within the range that the turret can rotate to, used by driver
        boolean inRotationRange=true;
        return inRotationRange;
    }
    
    public boolean inRange() {
        //returns whether or not the target is within the range that the shooter can shoot, used by driver
        boolean inRange=true;
        return inRange;
    }
    public void unJam() {
        //reverses shooter motors to attempt to unjam the shooter
    }
    // The exception to this is a general-functionality stop() method.
}
