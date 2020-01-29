package com.spartronics4915.frc2020;

public final class Constants {

    public static final class Climber {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
        public static final double kExtendSpeed = 1.0;
        public static final double kWinchSpeed = 1.0;
        public static final boolean kStalled = true;
    }
    
    public static final class Indexer {
        public static final class Spinner {
            public static final int kMotorId = -1;
            public static final double kP = 1;
            public static final double kD = 1;
            public static final double kConversionRatio = 1;
        }
        public static final class Loader {
            public static final int kMotor = -1;
            public static final double kP = 1;
            public static final double kD = 1;
            public static final double kConversionRatio = 1;
        }
        public static final int kProxSensorId = -1;
        public static final int kOpticalFlagId = -1;
    }   

    public static final class Launcher {
        public static final int kFlywheelMasterID = -1;
        public static final int kFlywheelFollowerID = -1;
        public static final int kAngleAdjusterID = -1;
        public static final int kTurretID = -1;
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

        public static final double kExtendMotorSpeed = 0.5;
        public static final double kSpinMotorSpeed = 0.5;
    }
}
