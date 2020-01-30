package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static final class Climber {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
        public static final double kExtendSpeed = 1.0;
        public static final double kWinchSpeed = 1.0;
        public static final boolean kStalled = true;
    }
    
    public static final class Indexer {
        public static final int kSpinnerId = -1;
        public static final int kLoaderId = -1;
        public static final int kProxSensorId = -1;
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

    public static final class Drive {
        public static final int kRightDriveMaster = 1;
        public static final int kRightDriveFollower = 2;
        public static final int kLeftDriveMaster = 3;
        public static final int kLeftDriveFollower = 4;

        public static final double kWheelDiameter = Units.inchesToMeters(8);
        public static final double kTrackWidthMeters = 1;
        public static final double kScrubFactor = 1; //TODO: characterize
        public static final int kNativeUnitsPerRevolution = 1; //TODO: get ratio

        public static final double kRobotMassKg = 1;
        public static final double kMoi = 1;

        //TODO: characterize
        public static final double kRightS = 1;
        public static final double kRightV = 1;
        public static final double kRightA = 1;
        
        public static final double kLeftS = 1;
        public static final double kLeftV = 1;
        public static final double kLeftA = 1;
    }

    public static final class Trajectory {
        public static final double kStartVelocityMetersPerSec = 0;
        public static final double kEndVelocityMetersPerSec = 0;
        public static final double kMaxVelocityMetersPerSec = 12;
        public static final double kMaxAccelerationMeterPerSecSq = 1;

        public static final Pose2d kStartPointLeft = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); //TODO: get world coordinates for start point left
        public static final Pose2d kStartPointMiddle = new Pose2d(50, 0, Rotation2d.fromDegrees(0)); //TODO: get world coordinates for start point middle
        public static final Pose2d kStartPointRight = new Pose2d(100, 0, Rotation2d.fromDegrees(0)); //TODO: get world coordinates for start point right
    }

    public static final class Estimator {
        public static final Pose2d kCameraOffset = new Pose2d();
        public static final double kMeasurementCovariance = 1;
    }
}
