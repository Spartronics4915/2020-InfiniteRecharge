package com.spartronics4915.frc2020;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;

import com.revrobotics.ColorMatch;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.TriFunction;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public final class Constants
{
    public static String sConfig;
    static  // check our machine id so subsystems can support multiple configs
    {
        sConfig = "default";
        Path machineIDPath = FileSystems.getDefault().getPath(System.getProperty("user.home"),
            "machineid");
        try
        {
            sConfig = Files.readString(machineIDPath).trim().toLowerCase();
        }
        catch (IOException e)
        {
        }
        Logger.notice("Running on machineconfig " + sConfig + " constants");
        Logger.notice("Running WPILibVersion " + WPILibVersion.Version);
    }

    public static final class Climber
    {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
        public static final double kExtendSpeed = 1.0; // XXX: test
        public static final double kWinchSpeed = -0.85; // XXX: test
        public static final double kRetractSpeed = -1.0; // XXX: test
        public static final double kReverseWinchSpeed = 1.0; // XXX: test
        public static final double kStallThreshold = 90.0; // FIXME: stand-in value
        public static final double kSecondaryStallThreshold = 5.0;
        public static final double kTimerExtenderMin = 3.0; // FIXME: stand-in values
        public static final double kTimerExtenderMax = 5.0;
        public static final boolean kStalled = true; // XXX: test
    }

    public static final class Indexer
    {
        public static final class Spinner
        {
            public static final int kMotorId = 9;
            public static final double kVelocityP = 0; // FIXME: stand-in values
            public static final double kVelocityD = 0;
            public static final double kPositionP = 0.005;
            public static final double kPositionD = 0;
            public static final double kConversionRatio = 1.0 / (187.0/20.0*9.0);
            public static final double kMaxVelocity = 1;
            public static final double kMaxAcceleration = 0.5;
            public static final double kStallThreshold = 90.0; // FIXME: stand-in values

            /** Degrees */
            public static final double kPositionTolerance = 2.0;
        }

        public static final class Loader
        {
            public static final int kMotorId = 10;
            public static final double kVelocityP = 1; // FIXME: stand-in values
            public static final double kVelocityD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
        }
        public static final class Transfer
        {
            public static final int kMotorId = 11;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = -1.0;
        }

        public static final int kLimitSwitchId = 8; // Digital
        public static final int kSlotProxSensorId = 4; // Digital
        public static final int kIntakeSensorId = 5; // Digital
    }

    public static final class Intake
    {
        public static final int kHarvestMotorId = 12;
        public static final int kProximitySensorId = 5; // Digital

        public static final double kHarvestSpeed = 0.5; // XXX: test
        public static final double kEjectSpeed = -0.5;  // XXX: test
    }

    public static final class Launcher
    {
        public static final int kFlywheelMasterId = 7; // CHANGE TO 7
        public static final int kFlywheelFollowerId = -1; // Solid brass
        public static final int kAngleAdjusterMasterId = 0; // PWM
        public static final int kAngleAdjusterFollowerId = 1; // PWM
        public static final int kTurretId = 8;
        public static final int kTurretPotentiometerId = 0; // Analog

        // XXX: consider whether to adopt CamToField2020, we currently have 
        // competing implementations.
        public static final Pose2d kRobotToTurret = new Pose2d(Units.inchesToMeters(-3.72), 
                                                    Units.inchesToMeters(5.264), 
                                                    Rotation2d.fromDegrees(180.0));
    
        // https://docs.wpilib.org/en/latest/docs/software/advanced-control/controllers/feedforward.html#simplemotorfeedforward
        public static final double kP = 0.05;
        public static final double kS = 0.0286; // 0.0654;
        public static final double kV = 7.86; // 7.18;
        public static final double kA = 5.16;

        public static final double kTurretP = 0.1;
        public static final double kTurretI = 0;
        public static final double kTurretD = 0.002;

        // Vals for interpolating lookup table, Distance units is in feet
        public static final double[] kDistanceTable = new double[]{3.0, 4.0, 4.5, 5.0};
        public static final double[] kAngleTable = new double[]{16.0, 20.0, 19.0, 24.0};
        public static final double[] kRPSTable = new double[]{38.0, 41.5, 41.0, 45.0};
        public static final int kLookupTableSize = kDistanceTable.length;

        /** RPS */
        public static final double kFlywheelVelocityTolerance = 1.0;
        public static final double kMaxRPS = 90.0; // Reasonable guess
        public static final double kMaxAngleDegrees = 30;
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(30.0);

        public static Pose2d goalLocation = null;
        //unit is feet
        public static final double MaxShootingDistance = 100;// FIXME: Figure out max distance
        //unit is feet
        public static final double MinShootingDistance = 0;// FIXME: Figure out min distance
        public static double kTurretStallAmps = 2.0;//Stand in Value
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

        public static final double kRaiseSpeed = -0.25;
        public static final double kLowerSpeed = 0.25;
        public static final double kSpinSpeed = 0.5; // XXX: test
        public static final double kConfidenceMinimum = 0.3; // FIXME: almost certainly incorrect

        public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114); // XXX: test these values in a variety of conditions
        public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }

    public static final class Drive
    {
        public static final TriFunction<Integer, SensorModel, Integer, SpartronicsMotor> kDriveMotorConstructor;

        public static final boolean kRightOutputInverted;
        public static final boolean kRightFollowerOutputInverted;
        public static final boolean kLeftOutputInverted;
        public static final boolean kLeftFollowerOutputInverted;

        public static final int kRightDriveMaster = 1;
        public static final int kRightDriveFollower = 2;
        public static final int kLeftDriveMaster = 3;
        public static final int kLeftDriveFollower = 4;

        public static final double kWheelDiameter;
        public static final double kTrackWidthMeters;
        public static final double kScrubFactor;
        public static final double kNativeUnitsPerRevolution;
        public static final double kSlowModeMultiplier;

        public static final double kRobotMassKg = 1;
        public static final double kMoi = 1;

        public static final double kP = 0.01;

        // TODO: characterize
        public static final double kRightS;
        public static final double kRightV;
        public static final double kRightA;

        public static final double kLeftS;
        public static final double kLeftV;
        public static final double kLeftA;

        public static final int kPigeonId;

        // Initialize blank fields that are robot-specific here
        static
        {
            switch (sConfig)
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
                    kSlowModeMultiplier = 0.5;
                    kLeftOutputInverted = true;
                    kLeftFollowerOutputInverted = false;
                    kRightOutputInverted = true;
                    kRightFollowerOutputInverted = false;

                    kPigeonId = -1;
                    kDriveMotorConstructor = SpartronicsSRX::makeMotor;
                    break;
                default:
                    kTrackWidthMeters = 26.75;
                    kWheelDiameter = Units.inchesToMeters(8);
                    kScrubFactor = 1;
                    kLeftS = 1;
                    kLeftV = 1;
                    kLeftA = 1;
                    kRightS = 1;
                    kRightV = 1;
                    kRightA = 1;
                    kNativeUnitsPerRevolution = 10.71;
                    kSlowModeMultiplier = 0.5;
                    kLeftOutputInverted = true;
                    kLeftFollowerOutputInverted = true;
                    kRightOutputInverted = false;
                    kRightFollowerOutputInverted = false;

                    kPigeonId = 1;
                    kDriveMotorConstructor = SpartronicsMax::makeMotor;
                    break;
            }
        }
    }

    public static final class Trajectory
    {
        public static final double kStartVelocityMetersPerSec = 0;
        public static final double kEndVelocityMetersPerSec = 0;
        public static final double kMaxVelocityMetersPerSec = 1;
        public static final double kMaxAccelerationMeterPerSecSq = .1;

        public static final Pose2d kStartPointLeft = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(138), Rotation2d.fromDegrees(180));
        public static final Pose2d kStartPointMiddle = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-65), Rotation2d.fromDegrees(180));
        public static final Pose2d kStartPointRight = new Pose2d(Units.inchesToMeters(508),
            Units.inchesToMeters(-138), Rotation2d.fromDegrees(180));
    }

    public static final class Estimator
    {
        // XXX: consider whether to adopt CamToField2020, we currently
        //  have competing implementations. 
        // If new measurements for Vision or Turret mounting are obtained,
        // please also update CoordSysMgr20202.java.
        public static final double kMeasurementCovariance = 0.001;
        public static final Pose2d kSlamraToRobot = new Pose2d(Units.inchesToMeters(-6.4375), 
                                                            Units.inchesToMeters(10.625), 
                                                            Rotation2d.fromDegrees(90));
    }

    public static final class Vision
    {
        /* Camera+mount geometry is located in CamToField2020.java */

        /* Raspi/Vision server status on /Vision namespace ------------------*/
        public static final String kTurretTargetTable = "/Vision/Target";
        public static final String kTargetResultKey = "/Vision/Target/Result"; /* from raspi */

        /* Vision subsystem status under /SmartDashboard/Vision namespace ----*/
        public static final String kPoseEstimateKey = "PoseEstimate";
        public static final String kPoseErrorKey = "PoseError";
        public static final String kPoseLatencyKey = "Latency";
        public static final String kOurGoalEstimateKey = "OurGoal";
        public static final String kTheirGoalEstimateKey = "OpponentGoal";
        public static final String kStatusKey = "Status";
        public static final String kLEDRelayKey = "LEDRelay";

        public static final double kGoalHeight = 8*12 + 2.25; // 98.25in

        // We assume here that the robot odometry is alliance-sensitive.
        // When we're on the Blue alliance, coords are
        //      [0, xsize] x [yhalfsize, -yhalfsize]
        // When we're on the Red alliance, coords are
        //      [xsize, 0] x [-yhalfsize, yhalfsize]
        // Given this behavior, we characterize Goals in our-alliance-relative
        // terms.
        public static final double[] kOpponentGoalCoords = {0, 67.5, kGoalHeight};
        public static final double[] kAllianceGoalCoords = {628, -67.5, kGoalHeight};

        public static final int kLEDRelayPin = 0; // Relay, not DIO pin!!!
    }
}
