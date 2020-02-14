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
    /* static class "constructor" - shared by inner classes */
    static public String sConfig = "default";
    static
    {
        // make available the contents of the per-roborio "machineid" file.
        // this allows us to tailor various constants according to the
        // configuration and hopefully reduce console spewage.
        String msg;
        try
        {
            Path machineIDPath = FileSystems.getDefault().getPath(
                                    System.getProperty("user.home"), "machineid");
            sConfig = Files.readString(machineIDPath).trim().toLowerCase();
            msg = "(via machineid)";
        }
        catch (IOException e)
        {
            msg = "(no machineid)";
        }
        Logger.notice("Running with " + sConfig + " configuration " + msg);
    }

    public static final class Climber
    {
        public static final int kLiftMotorId = 5;
        public static final int kWinchMotorId = 6;
        public static final double kExtendSpeed = 1.0; // XXX: test
        public static final double kWinchSpeed = 1.0; // XXX: test
        public static final double kRetractSpeed = -1.0; // XXX: test
        public static final double kReverseWinchSpeed = -1.0; // XXX: test
        public static final double kStallThreshold = 10.0; // FIXME: stand-in value
        public static final double kTimerExtenderMin = 3.0; // FIXME: stand-in values
        public static final double kTimerExtenderMax = 5.0;
        public static final boolean kStalled = true; // XXX: test
    }

    public static final class Indexer
    {
        public static final class Spinner
        {
            public static final int kMotorId = 9;
            public static final double kVelocityP = 1; // FIXME: stand-in values
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
            public static final double kVelocityP = 1; // FIXME: stand-in values
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
        }
        public static final class Transfer
        {
            public static final int kMotorId = 11;
            public static final double kVelocityP = 1; // FIXME: stand-in values
            public static final double kVelocityD = 1;
            public static final double kPositionP = 1;
            public static final double kPositionD = 1;
            public static final double kConversionRatio = 1;
            public static final double kSpeed = 1;
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
        public static final int kFlywheelMasterId = 7;
        public static final int kFlywheelFollowerId = -1; // Solid brass
        public static final int kAngleAdjusterMasterId = 0; // PWM
        public static final int kAngleAdjusterFollowerId = 1; // PWM
        public static final int kTurretId = 8;
        public static final int kTurretPotentiometerId = 0; // Analog

        // TODO: Find translation of turret from the center of the robot
        public static final Pose2d kTurretOffset = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));

        public static final double kP = 0.02; // FIXME: currently values for non-final launcher
        public static final double kS = 0.0634; // 0.0654;
        public static final double kV = 7.23; // 7.18;
        public static final double kA = 5.07;

        public static final double kTurretP = 0; // FIXME: stand-in values
        public static final double kTurretI = 0;
        public static final double kTurretD = 0;

        // Vals for interpolating lookup table
        public static final int LookupTableSize = 0; // ???
        public static final double[] DistanceTable = null;
        public static final double[] AngleTable = null;
        public static final double[] RPSTable = null;

        public static final double kMaxRPS = 90.0; // Reasonable guess
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(30.0);

        public static Pose2d goalLocation = null;
    }

    /**
     * Specify robot/testbed-specific device configurations so we minimize
     * console spewage associated with missing joysticks.
     * 
     * Optionally declare button binding here.
     */
    public static final class OI
    {
        public static int kJoystickPort = 0; /* driverstation usb port */
        public static int kButtonBoardPort = 1;

        // Buttons start at button #1 so button0 is invalid
        public static int[] kJoystickButtonMap = {-1,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
        public static int[] kButtonBoardButtonMap = {-1,1,2,3,4,5,6,7,8,9,10,11,12,13,14}; 
        private static int[][] mOIMap = {kJoystickButtonMap, kButtonBoardButtonMap};

        static
        {
            // default/initial config is based on roborio/machineid
            applyConfig(sConfig); 
        }

        // reduce default availability of port or buttons here.
        public static void applyConfig(final String str)
        {
            switch (str)
            {
                case "test chassis":
                    // test chassis has no button board
                    kButtonBoardPort = 0; 
                    break;
                default:
                    break;
            }
        }

        /**
         * returns a new button index for the requested button
         * @param port specifies which usb port housing the button
         * @param id specifies a button id.
         * @return -1 means no mapping, otherwise return is buttonid [1-N]
         */
        public static int getRemappedButton(int port, int id)
        {
            if(port < 0) return -1;
            if(mOIMap.length >= port) return -1; // invalid port

            int[] map = mOIMap[port];
            if(id >= map.length) return -1; // invalid id

            return mOIMap[port][id];
        }
    }

    public static final class PanelRotator
    {
        public static final int kLimitSwitchDownId = 6;
        public static final int kOpticalFlagUpId = 7;
        public static final int kSpinMotorId = 13;
        public static final int kRaiseMotorId = 14;

        public static final double kRaiseSpeed = 0.5; // XXX: test
        public static final double kLowerSpeed = -0.5;
        public static final double kSpinSpeed = 0.5; // XXX: test
        public static final double kConfidenceMinimum = 0.3; // FIXME: almost certainly incorrect

        public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114); // XXX: test these values in a variety of conditions
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
        public static final Pose2d kSlamraToRobot = new Pose2d(-0.390525, 0, new Rotation2d());
        public static final Pose2d kVisionToRobot = new Pose2d(0.390525, 0, Rotation2d.fromDegrees(0));
        public static final double kMeasurementCovariance = 0.001;
    }

    public static final class Vision
    {
        /* Camera mount geometry is located in CamToField2020.java */

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
    }
}
