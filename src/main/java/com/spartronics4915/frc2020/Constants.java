package com.spartronics4915.frc2020;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public final class Constants {
    public static final class Indexer {
        public static final int kSpinnerId = -1;
        public static final int kLiftId = -1;
        public static final int kProxSensorId = -1;
    }   

    public static final class Climber {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
    }

    public static final class OI {
        public static final int kJoystickId = 0;
        public static final int kButtonBoardId = 1;
    }

    public static final class PanelRotator {
        public static final int kBeamSensorUpID = -1;
        public static final int kBeamSensorDownID = -1;

        public static final int kExtendMotorID = -1;
        public static final int kSpinMotorID = -1;

        //TODO:This is bad and does not work. Needs to be implemented in a functional way.
        public static final Port kColorSensorLeftID = I2C.Port.kOnboard;
        public static final Port kColorSensorRightID = I2C.Port.kOnboard;

        public static final double kExtendMotorSpeed = 0.5;
        public static final double kSpinMotorSpeed = 0.5;
    }

}
