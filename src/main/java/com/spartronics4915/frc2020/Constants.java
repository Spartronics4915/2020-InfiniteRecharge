package com.spartronics4915.frc2020;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public final class Constants {
    public static final int kPanelRotatorBeamSensorUpID = -1;
    public static final int kPanelRotatorBeamSensorDownID = -1;

    public static final int kPanelRotatorExtendMotorID = -1;
    public static final int kPanelRotatorSpinMotorID = -1;

    //TODO:This is bad and does not work. Needs to be implemented in a functional way.
    public static final Port kPanelRotatorColorSensorLeftID = I2C.Port.kOnboard;
    public static final Port kPanelRotatorColorSensorRightID = I2C.Port.kOnboard;

    public static final double kPanelRotatorExtendMotorSpeed = 0.5;
    public static final double kPanelRotatorSpinMotorSpeed = 0.5;
}