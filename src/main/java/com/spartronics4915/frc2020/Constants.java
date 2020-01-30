package com.spartronics4915.frc2020;

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
        public static final int kSpinnerId = -1;
        public static final int kLoaderId = -1;
        public static final int kProxSensorId = -1;
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
        public static final int kFlywheelMasterID = 7;
        public static final int kFlywheelFollowerID = -1;
        public static final int kAngleAdjusterMasterID = -1;
        public static final int kAngleAdjusterFollowerID = -1;
        public static final int kTurretID = 8;
        public static final int kTurretPotentiometerID = 9;
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
