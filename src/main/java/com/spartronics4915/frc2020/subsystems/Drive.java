package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.sensors.SpartronicsPigeon;
import com.spartronics4915.lib.hardware.sensors.SpartronicsXRS450;
import com.spartronics4915.lib.math.twodim.physics.DCMotorTransmission;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.subsystems.drive.AbstractDrive;

import static com.spartronics4915.frc2020.Constants.Drive.*;

public class Drive extends AbstractDrive
{
    private Launcher mLauncher;

    public Drive(Launcher launcher)
    {
        super(
            kDriveMotorConstructor.apply(
                kLeftDriveMaster,
                SensorModel.fromWheelDiameter(kWheelDiameter, kNativeUnitsPerRevolution),
                kLeftDriveFollower),
            kDriveMotorConstructor.apply(
                kRightDriveMaster,
                SensorModel.fromWheelDiameter(kWheelDiameter, kNativeUnitsPerRevolution),
                kRightDriveFollower),
            kPigeonId != -1 ? new SpartronicsPigeon(kPigeonId): new SpartronicsXRS450(),
            new DifferentialDrive(kRobotMassKg, kMoi, 1, kWheelDiameter / 2, kTrackWidthMeters,
                new DCMotorTransmission(kWheelDiameter / 2, kRobotMassKg, kLeftS, kLeftV, kLeftA),
                new DCMotorTransmission(kWheelDiameter / 2, kRobotMassKg, kRightS, kRightV, kRightA))); // end super()

        mLauncher = launcher;

        mLeftMotor.setOutputInverted(kLeftOutputInverted);
        mRightMotor.setOutputInverted(kRightOutputInverted);

        mLeftMotor.getFollower().setOutputInverted(kLeftFollowerOutputInverted);
        mRightMotor.getFollower().setOutputInverted(kRightFollowerOutputInverted);

        mLeftMotor.setStatorCurrentLimit(30);
        mRightMotor.setStatorCurrentLimit(30);
        mLeftMotor.getFollower().setStatorCurrentLimit(30);
        mRightMotor.getFollower().setStatorCurrentLimit(30);

        mLeftMotor.setVelocityGains(kP, 0);
        mRightMotor.setVelocityGains(kP, 0);
    }

    @Override
    public double getTurretAngle() // XXX: why is getTurretAngle in Drive, of all places?
    {
        return mLauncher.getTurretDirection().getDegrees();
    }
}
