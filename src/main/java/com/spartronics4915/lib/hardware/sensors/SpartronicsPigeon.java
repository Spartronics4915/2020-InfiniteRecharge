package com.spartronics4915.lib.hardware.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;

public class SpartronicsPigeon implements SpartronicsIMU {

    PigeonIMU mIMU;

    public SpartronicsPigeon(int canID) {
        PigeonIMU imu = new PigeonIMU(canID);
        mIMU = imu;
    }
    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(mIMU.getFusedHeading());
    }

}