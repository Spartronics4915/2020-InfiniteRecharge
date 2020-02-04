package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;

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
        public static final double kRetractSpeed = -1.0;
        public static final double kReverseWinchSpeed = -1.0;
        public static final double kStallThreshold = 10.0;
        public static final boolean kStalled = true;
    }

    public static final class Indexer
    {
        public static final class Spinner
        {
            public static final int kMotorId = 9;
            public static final double kVelocityP = 1;
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kMaxVelocity = 1;
            public static final double kMaxAcceleration = 1;
        }

        public static final class Loader
        {
            public static final int kMotorId = 10;
            public static final double kVelocityP = 1;
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
        }

        public static final int kProxSensorId = 4; // DIO4 // Proximity Sensor (index slot one)
        public static final int kOpticalFlagId = 2; // A2 // Optical Flag for Zeroing
        public static final boolean kOpticalFlagReversed = false; // Whether or not the optical flag
                                                                  // is inverted
        public static final int kIntakeSensorId = -1; // 
    }

    public static final class Intake
    {
        public static final int kHarvestMotorId = 12;
        public static final double kHarvestSpeed = 0.5;
        public static final double kEjectSpeed = -0.5;
    }

    public static final class Launcher
    {
        public static final int kFlywheelMasterId = 7;
        public static final int kFlywheelFollowerId = -1; // Solid brass
        public static final int kAngleAdjusterMasterId = 0; // PWM0
        public static final int kAngleAdjusterFollowerId = 1; // PWM1
        public static final int kTurretId = 8;
        public static final int kTurretPotentiometerId = 2; // A2
    }

    public static final class OI
    {
        public static final int kJoystickId = 0;
        public static final int kButtonBoardId = 1;
    }

    public static final class PanelRotator
    {
        public static final int kBeamSensorUpId = 2; // TODO: take up issue with electronics over these not being on the control map
        public static final int kBeamSensorDownId = 3;

        public static final int kExtendMotorId = 13;
        public static final int kSpinMotorId = 14;

        public static final double kRaiseSpeed = 0.5;
        public static final double kLowerSpeed = -0.5;
        public static final double kSpinMotorSpeed = 0.5;
    }

    public static final class Drive
    {
        public static final int kRightDriveMaster = 1;
        public static final int kRightDriveFollower = 2;
        public static final int kLeftDriveMaster = 3;
        public static final int kLeftDriveFollower = 4;

        public static final double kWheelDiameter;
        public static final double kTrackWidthMeters;
        public static final double kScrubFactor;
        public static final int kNativeUnitsPerRevolution = 1; // TODO: get ratio

        public static final double kRobotMassKg = 1;
        public static final double kMoi = 1;

        // TODO: characterize
        public static final double kRightS;
        public static final double kRightV;
        public static final double kRightA;

        public static final double kLeftS;
        public static final double kLeftV;
        public static final double kLeftA;

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
                    kTrackWidthMeters = Units.inchesToMeters(23.75);
                    kWheelDiameter = Units.inchesToMeters(6.2554245800704);
                    kScrubFactor = 1.063;
                    kLeftS = 0.6995;
                    kLeftV = 0.2066;
                    kLeftA = 0.0107;
                    kRightS = 0.6815;
                    kRightV = 0.2194;
                    kRightA = 0.0340;
                    break;
                default:
                    kTrackWidthMeters = 1; // TODO: find track width
                    kWheelDiameter = Units.inchesToMeters(8);
                    //TODO characterize
                    kScrubFactor = 1;
                    kLeftS = 1;
                    kLeftV = 1;
                    kLeftA = 1;
                    kRightS = 1;
                    kRightV = 1;
                    kRightA = 1;
                    break;
            }
        }
    }

    public static final class Trajectory
    {
        public static final double kStartVelocityMetersPerSec = 0;
        public static final double kEndVelocityMetersPerSec = 0;
        public static final double kMaxVelocityMetersPerSec = 1;
        public static final double kMaxAccelerationMeterPerSecSq = 1;

        public static final Pose2d kStartPointLeft = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(138), Rotation2d.fromDegrees(180));
        public static final Pose2d kStartPointMiddle = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-54), Rotation2d.fromDegrees(180));
        public static final Pose2d kStartPointRight = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-138), Rotation2d.fromDegrees(180));
    }

    public static final class Estimator
    {
        public static final Pose2d kCameraOffset = new Pose2d();
        public static final double kMeasurementCovariance = 1;
    }
}
