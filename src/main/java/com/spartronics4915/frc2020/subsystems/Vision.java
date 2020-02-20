package com.spartronics4915.frc2020.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.CoordSysMgr2020;
import com.spartronics4915.lib.math.threedim.Vec3;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Vision subsystem has these responsibilities 
 * 
 * - to listen for vision coprocessor results and use it to estimate
 *   robot pose.  NB: this estimate is for a moment in the past associated
 *   with the timestamp delivered with the vision target position.
 * - to turn on & off the vision LED
 * 
 * We estimate the robot pose whenever we receive a vision target update from 
 * raspi. This pose should be delivered to the CoordSysManager. 
 * NB: there's an alternate operating mode where we the raspi-Vision 
 * code computes/estimates robot pose. In that case our job would be
 * to ensure that all required robot state would be presented to
 * the network tables.
 */

public class Vision extends SpartronicsSubsystem
{
    /* public interfaces ------------------------------------*/
    public class VisionEvent implements Runnable
    {
        public Pose2d mVisionEstimate=null;
        public double mTimestamp=0;

        public void run()
        {
        }; // override me
    }

    /* member variables -------------------------------------*/
    private final RobotStateMap mOfficialRSM;
    private final CoordSysMgr2020 mCoordSysMgr;
    private final NetworkTableInstance mNetTab;
    private List<VisionEvent> mListeners;
    private Deque<RobotStateMap.State> mVisionEstimates;
    private final Relay mLEDRelay; 
    private final Launcher mLauncher;

    /* NB: our targets are measured in inches, while RobotStateMap
     * operates in meters!
     */
    private final Vec3 mOurTarget = new Vec3(Constants.Vision.kAllianceGoalCoords);
    private final Vec3 mOpponentTarget = new Vec3(Constants.Vision.kOpponentGoalCoords);

    /**
     * Vision subsystem needs read-only access to RobotStateEstimator and
     * Launcher subsystem.
     * @param rse
     * @param launcherSubsys
     */
    public Vision(RobotStateEstimator rse, Launcher launcher)
    {
        this.mOfficialRSM = rse.getCameraRobotStateMap();
        this.mLauncher = launcher;
        this.mCoordSysMgr = new CoordSysMgr2020(); // our private copy

        this.mNetTab = NetworkTableInstance.getDefault();
        this.mNetTab.addEntryListener(Constants.Vision.kTargetResultKey, 
                                    this::visionTargetUpdate,
                                    EntryListenerFlags.kUpdate);
        this.mListeners = new ArrayList<VisionEvent>();
        this.mVisionEstimates = new ArrayDeque<RobotStateMap.State>();
        this.dashboardPutString(Constants.Vision.kStatusKey, "ready+waiting");

        this.mLEDRelay = new Relay(Constants.Vision.kLEDRelayPin);
        mLEDRelay.set(Relay.Value.kOn);
        /// XXX: set the relay into a known/desired state!
        this.dashboardPutString(Constants.Vision.kLEDRelayKey, 
                            this.mLEDRelay.get().toString());
    }

    /* VisionEvents -----------------------------------------------*/
    /**
     * register interest in vision events - caller may subclass
     * VisionEvent.  Alternative is just to request access to
     * the mVisionEstimates queue.
     * @param l - 
     */
    public void registerTargetListener(VisionEvent l)
    {
        this.mListeners.add(l);
    }

    /**
     * @return  Vision's "nearly independent" table of robot state estimations.
     * Note that only position reflects our computation.  The heading,  velocity
     * an acceleration are interpolated from RobotStateEstimator's tables.
     */
    public Deque<RobotStateMap.State> getVisionEstimates()
    {
        return this.mVisionEstimates; // nb: is there a threading issue here?
    }

    /* LED --------------------------------------------------------*/
    public boolean isLEDOn()
    {
        return (mLEDRelay.get() == Relay.Value.kForward);
    }

    public void setLED(boolean onoff)
    {
        mLEDRelay.set(onoff ? Value.kForward : Value.kOff);
    }

    /**
     * Nettabcallback. Invoked when robot receives target updates from vision.
     * @param event - information about the entry that triggered the callback.
     */
    private void visionTargetUpdate(EntryNotification event)
    {
        NetworkTableValue v = event.getEntry().getValue();
        if (v.isString())
        {
            String val = v.getString();
            // this.logInfo("Turret Target received " + val);
            // here we parse the string and evaluate the field
            // expect a string of the form:
            // "-33.35 -35.50 -100 22.35557 [turretAngle]" (camx, camy, camz, timestamp, turretAngle)
            // Turret angle may be delivered since its value in the past
            // is required to estimate robot pose.  If vision doesn't deliver
            // it, we'll either accept error, assume that the turret doesn't
            // move or need to extend RobotStateEstimator.state to include this
            // information.

            String[] vals = val.split(" ");
            double camx = Double.parseDouble(vals[0]);
            double camy = Double.parseDouble(vals[1]);
            double camz = Double.parseDouble(vals[2]);
            double timestamp = Double.parseDouble(vals[3]);
            double turretAngle = (vals.length == 5) ? 
                Double.parseDouble(vals[4]) : mLauncher.getTurretDirection().getDegrees();
            Vec3 tgtInCam = new Vec3(camx, camy, camz);
            Vec3 tgtInRobot = this.mCoordSysMgr.camPointToRobot(tgtInCam);
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
            this.mCoordSysMgr.updateTurretAngle(turretAngle);
            this.mCoordSysMgr.updateRobotPose(robotHeading, tgtInRobot, fieldTarget);
            Vec3 robotPos = mCoordSysMgr.robotPointToField(Vec3.ZeroPt);
            // use robot's heading in our poseEstimate - remember to
            // convert from inches to meters.
            Pose2d poseEstimate = new Pose2d(Units.inchesToMeters(robotPos.a1), 
                                            Units.inchesToMeters(robotPos.a2), 
                                            r2d);
            Iterator<VisionEvent> it = this.mListeners.iterator();
            while (it.hasNext())
            {
                VisionEvent e = it.next();
                e.mVisionEstimate = poseEstimate;
                e.mTimestamp = timestamp;
                e.run();
            }
            var state = new RobotStateMap.State(poseEstimate, timestamp);
            mVisionEstimates.addLast(state);

            // now measure the distance between our estimate and the
            // official robot estimate (at timestamp).
            double derror = robotPos.subtract(Units.metersToInches(t2d.getX()), 
                                              Units.metersToInches(t2d.getY()), 
                                              0).length();
            this.dashboardPutNumber(Constants.Vision.kPoseErrorKey, derror);

            // NB: robotPos is in inches! (dashboard too!)
            String pstr = String.format("%g %g %g", robotPos.a1, robotPos.a2, robotHeading);
            this.dashboardPutString(Constants.Vision.kPoseEstimateKey, pstr);

            double delay = Timer.getFPGATimestamp() - timestamp;
            this.dashboardPutNumber(Constants.Vision.kPoseLatencyKey, delay);

            // Report the combination of official odometry and target
            // offset. This is a "forward" estimate of our well-known
            // landmarks.  Hopefully these will produce nearly constant
            // and correct (!) results.
            mCoordSysMgr.updateRobotPose(Units.metersToInches(t2d.getX()), 
                                        Units.metersToInches(t2d.getY()), 
                                        r2d.getDegrees());
            Vec3 tgtInField = mCoordSysMgr.camPointToField(tgtInCam);
            String key = (fieldTarget == this.mOurTarget) ? 
                        Constants.Vision.kOurGoalEstimateKey :
                        Constants.Vision.kTheirGoalEstimateKey;
            this.dashboardPutString(key, tgtInField.asPointString());
        }
        else
            this.logError("Vision target value must be a string");
    }

}
