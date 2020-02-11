package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.util.Logger;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;

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
        public static final double kTimerExtenderMin = 3.0;
        public static final double kTimerExtenderMax = 5.0;
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
        public static final class Transfer
        {
            public static final int kMotorId = 11;
            public static final double kVelocityP = 1;
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
        }

        public static final int kOpticalFlagId = 8; // Analog
        public static final int kSlotProxSensorId = 4; // Digital
        public static final int kIntakeSensorId = 5; // Digital
    }

    public static final class Intake
    {
        public static final int kHarvestMotorId = 12;
        public static final int kProximitySensorId = 5; // Digital

        public static final double kHarvestSpeed = 0.5;
        public static final double kEjectSpeed = -0.5;
    }

    public static final class Launcher
    {
        public static final int kFlywheelMasterId = 7;
        public static final int kFlywheelFollowerId = -1; // Solid brass
        public static final int kAngleAdjusterMasterId = 0; // PWM
        public static final int kAngleAdjusterFollowerId = 1; // PWM
        public static final int kTurretId = 8;
        public static final int kTurretPotentiometerId = 0;

        // TODO: Find translation of turret from the center of the robot
        public static final Pose2d kTurretOffset = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));

        // https://docs.wpilib.org/en/latest/docs/software/advanced-control/controllers/feedforward.html#simplemotorfeedforward
        public static final double kP = 0.00154;
        public static final double kS = 0.0638;
        public static final double kV = 0.121;
        public static final double kA = 0.0252;

        // Vals for interpolating lookup table
        public static final int LookupTableSize = 0;
        public static final double[] DistanceTable = null;
        public static final double[] AngleTable = null;
        public static final double[] RPSTable = null;

        public static final double kMaxRPS = 90.0;
    }

    public static final class OI
    {
        public static final int kJoystickId = 0;
        public static final int kButtonBoardId = 1;
    }

    public static final class PanelRotator
    {
        public static final int kLimitSwitchDownId = 6;
        public static final int kOpticalFlagUpId = 7;
        public static final int kSpinMotorId = 13;
        public static final int kRaiseMotorId = 14;

        public static final double kRaiseSpeed = 0.5;
        public static final double kLowerSpeed = -0.5;
        public static final double kSpinSpeed = 0.5;
        public static final double kConfidenceMinimum = 0.3;

        public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }

    public static final class Drive
    {
        public static final boolean kRightOutputInverted = true;
        public static final boolean kRightFollowerOutputInverted = true;
        public static final boolean kLeftOutputInverted = false;
        public static final boolean kLeftFollowerOutputInverted = false;

        public static final int kRightDriveMaster = 1;
        public static final int kRightDriveFollower = 2;
        public static final int kLeftDriveMaster = 3;
        public static final int kLeftDriveFollower = 4;

        public static final double kWheelDiameter;
        public static final double kTrackWidthMeters;
        public static final double kScrubFactor;
        public static final double kNativeUnitsPerRevolution;

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
                    kScrubFactor = 1.063;
                    kLeftS = 0.6995;
                    kLeftV = 0.2066;
                    kLeftA = 0.0107;
                    kRightS = 0.6815;
                    kRightV = 0.2194;
                    kRightA = 0.0340;
                    kWheelDiameter = Units.inchesToMeters(6);
                    kNativeUnitsPerRevolution = 1440.0;
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
                    kNativeUnitsPerRevolution = 10.71;
                    break;
            }
        }
    }

    public static final class Trajectory
    {
        public static final double kStartVelocityMetersPerSec = 0;
        public static final double kEndVelocityMetersPerSec = 0;
        public static final double kMaxVelocityMetersPerSec = .2;
        public static final double kMaxAccelerationMeterPerSecSq = .1;

        public static final Pose2d kStartPointLeft = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(138), Rotation2d.fromDegrees(180));
        public static final Pose2d kStartPointMiddle = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-54), Rotation2d.fromDegrees(180));
            //(x:508, y:-54, r:180) (x:400, y:-40, r:135)
        public static final Pose2d kStartPointRight = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-138), Rotation2d.fromDegrees(180));
    }

    public static final class Estimator
    {
        public static final Pose2d kCameraOffset = new Pose2d();
        public static final double kMeasurementCovariance = 0.001;
    }

    public static final class Vision
    {
        /* Camera mount geometry is located in CamToField2020.java */
        public static final String kTurretTargetKey = "/Vision/Target/Turret";
        public static final String kPoseBroadcastKey = "/Vision/Control/RobotPose";
    }
}
