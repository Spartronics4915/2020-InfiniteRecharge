package com.spartronics4915.frc2020.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.CamToField2020;
import com.spartronics4915.frc2020.subsystems.Launcher;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.math.twodim.geometry.*;
import com.spartronics4915.lib.math.threedim.*;

/**
 * The Vision subsystem has these responsibilities 
 * 
 * - to listen to launcher for turret position
 * - to listen for vision coprocessor results and use it to estimate
 *   robot pose.
 * - to deliver a new robot state estimate into our RobotStateMap 
 *   (or ultimately deliver it to the RobotStateEstimator).
 * 
 * issues:
 */
public class Vision extends SpartronicsSubsystem
{
    /* public interfaces ------------------------------------*/
    public class VisionEvent implements Runnable
    {
        public Pose2d mVisionEstimate;

        VisionEvent(Pose2d estimate)
        {
            this.mVisionEstimate = estimate;
        }

        public void run()
        {
        }; // override me
    }

    /* member variables -------------------------------------*/
    RobotStateMap mOfficialRSM, mVisionRSM;
    NetworkTableInstance mNetTab;
    CamToField2020 mCamToField;
    Launcher mLauncher;
    Vec3 mOurTarget = new Vec3(Constants.Vision.kAllianceGoalCoords);
    Vec3 mOpponentTarget = new Vec3(Constants.Vision.kOpponentGoalCoords);
    String mStatus;
    List<VisionEvent> mListeners;

    /**
     * Vision subsystem needs read-only access to RobotStateEstimator and
     * Launcher subsystem.
     * @param rse
     * @param launcherSubsys
     */
    public Vision(RobotStateEstimator rse, Launcher launcherSubsys)
    {
        this.mOfficialRSM = rse.getCameraRobotStateMap();
        this.mVisionRSM = new RobotStateMap();
        this.mLauncher = launcherSubsys;
        this.mNetTab = NetworkTableInstance.getDefault();
        this.mNetTab.addEntryListener(Constants.Vision.kTargetResultKey, this::turretTargetUpdate,
            EntryListenerFlags.kUpdate);
        this.mCamToField = new CamToField2020();
        this.mListeners = new ArrayList<VisionEvent>();
        this.setDefaultCommand(new ListenForTurretAndVision());
        this.dashboardPutString(Constants.Vision.kStatusKey, "ready+waiting");
    }

    public void registerTargetListener(VisionEvent l)
    {
        this.mListeners.add(l);
    }

    /**
     * @return  Vision's "nearly independent" table of robot state estimations.
     * Note that only position reflects our computation.  The heading,  velocity
     * an acceleration are interpolated from RobotStateEstimator's tables.
     */
    public RobotStateMap getRobotStateM0p()
    {
        return this.mVisionRSM;
    }

    /**
     * A currently-unused method that might be of use in the inverted scenario
     * where we expect the raspi-Vision code to compute robot pose.  Currently
     * we take the opposite tack: we esimate the pose whenever we receive a
     * vision target update from raspi.
     */
    public void broadcastState()
    {
        // Here we might compose an easily parsable string containing
        // current robot pose estimate and timestamp. We could
        // also include the current turret rotation (and elevation
        // if the camera is mounted thereupon).
        //
        // On the other hand, the RobotStateEstimator currently outputs:
        // RobotState/pose
        // RobotState/timeStamp.
        // If we can rely on the launcher to output:
        // Launcher/StaticPosition (xy, position relative to robot origin)
        // Launcher/StaticOrientation (roll, pitch, yaw)
        // Launcher/TurretAimAngle
        // Then we really have nothing to contribute yet.
    }

    /**
     * Nettabcallback. Invoked when robot receives target updates from vision.
     * @param event - information about the entry that triggered the callback.
     */
    private void turretTargetUpdate(EntryNotification event)
    {
        NetworkTableValue v = event.getEntry().getValue();
        if (v.isString())
        {
            String val = v.getString();
            // this.logInfo("Turret Target received " + val);
            // here we parse the string and evaluate the field
            // expect a string of the form:
            // "-33.35 -35.50 -100 22.35557" (camx, camy, camz, timestamp)

            String[] vals = val.split(" ");
            assert vals.length == 4;
            double camx = Double.parseDouble(vals[0]);
            double camy = Double.parseDouble(vals[1]);
            double camz = Double.parseDouble(vals[2]);
            double timestamp = Double.parseDouble(vals[3]);
            Vec3 tgtInCam = new Vec3(camx, camy, camz);
            Rotation2d turretDegrees = mLauncher.getTurretDirection();
            this.mCamToField.updateTurretAngle(turretDegrees);
            Vec3 tgtInRobot = this.mCamToField.camPointToRobot(tgtInCam);
            if (tgtInRobot.a1 <= 0)
                this.dashboardPutString(Constants.Vision.kStatusKey, "CONFUSED!!!");
            else
                this.dashboardPutString(Constants.Vision.kStatusKey, "active");

            // Now we have the target in robot-relative coordinates.
            // We know that the robot "sees" out its back-end, but the
            // robot can be oriented arbitrarily. We currently have two
            // targets at well-known field coordinates, one on each end of
            // the field, so here's the plan:
            // 1. consult the robot-state-estimator for the robot orientation
            // at timestamp.
            // 2. select the likely target
            // 3. use likely target to localize robot to field
            // 4. produce an updated estimate of robot location (trust RSE orientation)
            // 5. store this in our version of a RobotStateMap (at time)
            RobotStateMap.State officialState = mOfficialRSM.get(timestamp);
            Pose2d robotToField = officialState.pose;
            Translation2d t2d = robotToField.getTranslation();
            Rotation2d r2d = robotToField.getRotation();
            double robotHeading = r2d.getDegrees();
            Vec3 fieldTarget;
            // Our target is at field heading == -180 since the turret is
            // mounted on back.. 
            if ((robotHeading < 90 && robotHeading > -90) || robotHeading > 270)
            {
                // We're more likely to see theirs than ours.
                fieldTarget = this.mOpponentTarget;
            }
            else
            {
                // We're more likely to see ours than theirs.
                fieldTarget = this.mOurTarget;
            }

            // here is the inverse estimate -----------------------------
            mCamToField.updateRobotPose(robotHeading, tgtInRobot, fieldTarget);
            Vec3 robotPos = mCamToField.robotPointToField(Vec3.ZeroPt);
            // use robot's heading in our poseEsimtate
            Pose2d poseEstimate = new Pose2d(robotPos.a1, robotPos.a2, r2d);
            Iterator<VisionEvent> it = this.mListeners.iterator();
            while (it.hasNext())
            {
                VisionEvent e = it.next();
                e.mVisionEstimate = poseEstimate;
                e.run();
            }
            this.mVisionRSM.addObservations(timestamp, poseEstimate,
                officialState.integrationVelocity, officialState.predictedVelocity);

            // now measure the distance between our estimate and the
            // official robot estimate.
            double derror = robotPos.subtract(t2d.getX(), t2d.getY(), 0).length();
            this.dashboardPutNumber(Constants.Vision.kPoseErrorKey, derror);

            String pstr = String.format("%g %g %g", robotPos.a1, robotPos.a2, robotHeading);
            this.dashboardPutString(Constants.Vision.kPoseEstimateKey, pstr);

            double delay = Timer.getFPGATimestamp() - timestamp;
            this.dashboardPutNumber(Constants.Vision.kPoseLatencyKey, delay);

            // Let's report the combination of official odometry and target
            // offset. This is a "forward" estimate of our well-known
            // landmarks.  Hopefully these will produce nearly constant
            // and correct (!) results.
            mCamToField.updateRobotPose(t2d.getX(), t2d.getY(), r2d.getDegrees());
            Vec3 tgtInField = mCamToField.camPointToField(tgtInCam);
            String key = (fieldTarget == this.mOurTarget) ? 
                        Constants.Vision.kOurGoalEstimateKey :
                        Constants.Vision.kTheirGoalEstimateKey;
            this.dashboardPutString(key, tgtInField.asPointString());
        }
        else
            this.logError("Turret Target value must be a string");
    }

    /* private interfaces ---------------------------------------------*/
    /**
     * ListenForTurretAndVision is this subsystem's default command.
     */
    private class ListenForTurretAndVision extends CommandBase
    {
        public ListenForTurretAndVision()
        {
            this.addRequirements(Vision.this);
        }

        public boolean isFinished()
        {
            return false; // for clarity, we're always in this mode
        }
    }
}
