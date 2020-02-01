package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;

import edu.wpi.first.wpilibj.util.Units;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;

import com.spartronics4915.lib.util.Logger;

public final class Constants
{

    public static final class Climber
    {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
        public static final double kExtendSpeed = 1.0;
        public static final double kWinchSpeed = 1.0;
        public static final boolean kStalled = true;
    }

    public static final class Indexer
    {
        public static final class Spinner
        {
            public static final int kMotorId = 10;
            public static final double kVelocityP = 1;
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
        }

        public static final class Loader
        {
            public static final int kMotor = 11;
            public static final double kVelocityP = 1;
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
        }

        public static final int kProxSensorId = 4; // DIO4 // Proximity Sensor (index slot one)
        public static final int kOpticalFlagId = 2; // A2 // Optical Flag for Zeroing
        public static final boolean kOpticalFlagReversed = false; // Whether or not the optical flag is inverted
    }

    public static final class Intake
    {
        public static final int kHarvestMotorId = 12;
        public static final int kIngestMotorId = 13;
        public static final double kHarvestSpeed = 0.5;
        public static final double kIngestSpeed = 0.5;
    }

    public static final class Launcher
    {
        public static final int kFlywheelMasterId = 7;
        public static final int kFlywheelFollowerId = -1; // Solid brass
        public static final int kAngleAdjusterMasterId = 6;
        public static final int kAngleAdjusterFollowerId = 7;
        public static final int kTurretId = 8;
        public static final int kTurretPotentiometerId = 1;
        public static final double kP = 0.0154;
        public static final double kS = 0.0638;
        public static final double kV = 0.121;
        public static final double kA = 0.0252;
    }

    public static final class OI
    {
        public static final int kJoystickId = 1;
        public static final int kButtonBoardId = 1;
    }

    public static final class PanelRotator
    {
        public static final int kBeamSensorUpID = -1;
        public static final int kBeamSensorDownID = -1;

        public static final int kExtendMotorID = -1;
        public static final int kSpinMotorID = -1;

        public static final double kExtendMotorSpeed = 0.5;
        public static final double kSpinMotorSpeed = 0.5;
    }

    public static final class Drive
    {
        public static final int kRightDriveMaster = 1;
        public static final int kRightDriveFollower = 2;
        public static final int kLeftDriveMaster = 3;
        public static final int kLeftDriveFollower = 4;

        public static final double kWheelDiameter = Units.inchesToMeters(8);
        public static final double kTrackWidthMeters = 1;
        public static final double kScrubFactor = 1; // TODO: characterize
        public static final int kNativeUnitsPerRevolution = 1; // TODO: get ratio

        public static final double kRobotMassKg = 1;
        public static final double kMoi = 1;

        // TODO: characterize
        public static final double kRightS = 1;
        public static final double kRightV = 1;
        public static final double kRightA = 1;

        public static final double kLeftS = 1;
        public static final double kLeftV = 1;
        public static final double kLeftA = 1;
    }

    public static final class Trajectory
    {
        public static final double kStartVelocityMetersPerSec = 0;
        public static final double kEndVelocityMetersPerSec = 0;
        public static final double kMaxVelocityMetersPerSec = 12;
        public static final double kMaxAccelerationMeterPerSecSq = 1;

        // TODO: get world coordinates for start point left
        public static final Pose2d kStartPointLeft = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        // TODO: get world coordinates for start point middle
        public static final Pose2d kStartPointMiddle = new Pose2d(50, 0, Rotation2d.fromDegrees(0));
        // TODO: get world coordinates for start point right
        public static final Pose2d kStartPointRight = new Pose2d(100, 0, Rotation2d.fromDegrees(0));
    }

    public static final class Estimator
    {
        public static final Pose2d kCameraOffset = new Pose2d();
        public static final double kMeasurementCovariance = 1;
    }

    // Initialize blank fields that are robot-specific here
    static
    {
        String config = "default";
        Path machineIDPath = FileSystems.getDefault().getPath(System.getProperty("user.home"),
                "machineid");
        try
        {
            config = Files.readString(machineIDPath).trim().toLowerCase();
        }
        catch (IOException e)
        {
        }
        Logger.notice("Running on " + config + " constants");

        switch (config)
        {
            case "test chassis":
                // Put constants here
                break;
            case "default":
            case "real robot":
                // Or here
                break;
        }
    }
}
