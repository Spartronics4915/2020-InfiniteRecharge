package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/**
 * The Vision subsystem has two responsibilities 
 * 1. to broadcast (over networktables) the current timestamped robot pose 
 *    for consumption by interested vision coprocessors.
 * 2. to listen for vision coprocessor results and update the robot 
 *    odometry sensor-fusion database.
 * Consumption of the PnP results is entirely left up to the consumer.
 */
public class Vision extends SpartronicsSubsystem
{
    RobotStateEstimator mRSE;
    NetworkTableInstance mNetTab;

    public Vision(RobotStateEstimator rse)
    {
        this.mRSE = rse;
        this.mNetTab = NetworkTableInstance.getDefault();
        this.mNetTab.addEntryListener(Constants.Vision.kTurretTargetKey,
                                      this::turretTargetUpdate, 
                                      EntryListenerFlags.kUpdate);
    }

    public void BroadcastState()
    {
        // Here we might compose an easily parsable string containing
        // current robot pose estimate and timestamp. We could
        // also include the current turret rotation (and elevation
        // if the camera is mounted thereupon).
        //
        // On the other hand, the RobotStateEstimator currently outputs: 
        //    RobotState/pose 
        //    RobotState/timeStamp.
        // If we can rely on the launcher to output:
        //    Launcher/StaticPosition (xy, position relative to robot origin)
        //    Launcher/StaticOrientation (roll, pitch, yaw)
        //    Launcher/TurretAimAngle
        // Then we really have nothing to contribute yet.
    }

    // we're called when robot receives turret update
    private void turretTargetUpdate(EntryNotification event)
    {
        NetworkTableValue v = event.getEntry().getValue();
        if(v.isString())
        {
            this.logInfo("Turret Target received " + v.getString());
            // here we parse the string and evaluate the field
            // position of the target at the time the target was 
            // identified.
        }
        else
            this.logError("Turret Target value must be a string");
    }
}