package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.sensors.SpartronicsPigeon;
import com.spartronics4915.lib.hardware.sensors.SpartronicsXRS450;
import com.spartronics4915.lib.math.twodim.physics.DCMotorTransmission;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.subsystems.drive.AbstractDrive;

public class Drive extends AbstractDrive
{
    private Launcher mLauncher;

    public Drive(Launcher launcher)
    {
        super(
            SpartronicsMax.makeMotor(
                Constants.Drive.kLeftDriveMaster,
                SensorModel.fromWheelDiameter(
                    Constants.Drive.kWheelDiameter,
                    Constants.Drive.kNativeUnitsPerRevolution),
                Constants.Drive.kLeftDriveFollower),
            Constants.Drive.kDriveMotorConstructor.apply(
                Constants.Drive.kRightDriveMaster,
                SensorModel.fromWheelDiameter(
                    Constants.Drive.kWheelDiameter,
                    Constants.Drive.kNativeUnitsPerRevolution),
                Constants.Drive.kRightDriveFollower),
            Constants.Drive.kPigeonId != -1 ? new SpartronicsPigeon(Constants.Drive.kPigeonId)
                : new SpartronicsXRS450(),
            new DifferentialDrive(
                Constants.Drive.kRobotMassKg,
                Constants.Drive.kMoi,
                1,
                Constants.Drive.kWheelDiameter / 2,
                Constants.Drive.kTrackWidthMeters,
                new DCMotorTransmission(
                    Constants.Drive.kWheelDiameter / 2,
                    Constants.Drive.kRobotMassKg,
                    Constants.Drive.kLeftS,
                    Constants.Drive.kLeftV,
                    Constants.Drive.kLeftA),
                new DCMotorTransmission(
                    Constants.Drive.kWheelDiameter / 2,
                    Constants.Drive.kRobotMassKg,
                    Constants.Drive.kRightS,
                    Constants.Drive.kRightV,
                    Constants.Drive.kRightA))); // end super()

        mLauncher = launcher;

        mLeftMotor.setOutputInverted(Constants.Drive.kLeftOutputInverted);
        mRightMotor.setOutputInverted(Constants.Drive.kRightOutputInverted);

        mLeftMotor.getFollower().setOutputInverted(Constants.Drive.kLeftFollowerOutputInverted);
        mRightMotor.getFollower().setOutputInverted(Constants.Drive.kRightFollowerOutputInverted);

        mLeftMotor.setStatorCurrentLimit(30);
        mRightMotor.setStatorCurrentLimit(30);
        mLeftMotor.getFollower().setStatorCurrentLimit(30);
        mRightMotor.getFollower().setStatorCurrentLimit(30);

        mLeftMotor.setVelocityGains(Constants.Drive.kP, 0);
        mRightMotor.setVelocityGains(Constants.Drive.kP, 0);
    }

    @Override
    public double getTurretAngle()
    {
        return mLauncher.getTurretDirection().getDegrees();
    }
}
