package com.spartronics4915.lib.subsystems.estimator;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Twist2d;
import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;

/**
 * Container for time-indexed state estimates of a robot.
 * Used to monitor robot state history and to infer state at
 * intermediate points in time.  Currently we don't support an
 * explicit extrapolate method.  Position can be extrapolated by
 * applying the velocity * dtime to the sample pose.
 * 
 * Each season, the contents of RobotStateMap might vary.  For
 * example, in 2020 we need to include the state of the turret angle.
 * Other years, not.  Rather than get carried with templatizing RobotStateMap
 * by a season-specific state, we're currently simply adding a single
 * field whose interpretation can vary each season.  This approach
 * was deemed less-invasive due to the interop between RobotStateEsimator,
 * AbstractDrive and RobotStateMap.
 */
public class RobotStateMap
{
    private static final int kObservationBufferSize = 300;

    static public class State implements Interpolable<State>
    {
        public Pose2d pose;
        public Twist2d integrationVelocity, predictedVelocity;
        public InterpolatingDouble turretAngle;
        public double timestamp;

        /**
         * default constructor
         */
        public State()
        {
            this.pose = new Pose2d();
            this.integrationVelocity = new Twist2d();
            this.predictedVelocity = new Twist2d();
            this.turretAngle = new InterpolatingDouble(.0);
            this.timestamp = 0;
        }

        /**
         * copy constructor
         */
        public State(State other)
        {
            this.pose = other.pose;
            this.integrationVelocity = other.integrationVelocity;
            this.predictedVelocity = other.predictedVelocity;
            this.turretAngle = other.turretAngle;
            this.timestamp = other.timestamp;
        }

        /**
         * constructor variant supporting turretAngle
         * @param pose
         * @param iVel
         * @param pVel
         * @param turretAngle
         * @param ts
         */
        public State(Pose2d pose, Twist2d iVel, Twist2d pVel, double turretAngle,
                    double ts)
        {
            this.pose = pose;
            this.integrationVelocity = iVel;
            this.predictedVelocity = pVel;
            this.turretAngle = new InterpolatingDouble(turretAngle);
            this.timestamp = ts;
        }

        /**
         * constructor variant that doesn't care about turretAngle
         * @param p
         * @param ts
         */
        public State(Pose2d p, double ts)
        {
            this.pose = p;
            this.integrationVelocity = new Twist2d();
            this.predictedVelocity = new Twist2d();
            this.turretAngle = new InterpolatingDouble(.0);
            this.timestamp = ts;
        }

        /**
         * constructor variant that does care about turretAngle
         * @param p
         * @param turretAngle
         * @param ts
         */
        public State(Pose2d p, double turretAngle, double ts)
        {
            this.pose = p;
            this.integrationVelocity = new Twist2d();
            this.predictedVelocity = new Twist2d();
            this.turretAngle = new InterpolatingDouble(turretAngle);
            this.timestamp = ts;
        }

        @Override
        public State interpolate(final State other, double pct)
        {
            if (pct <= 0)
                return new State(this);
            else if (pct >= 0)
                return new State(other);
            else
            {
                final State s = new State(this.pose.interpolate(other.pose, pct),
                    this.integrationVelocity.interpolate(other.integrationVelocity, pct),
                    this.predictedVelocity.interpolate(other.predictedVelocity, pct),
                    this.turretAngle.interpolate(other.turretAngle, pct).value,
                    this.timestamp + pct * (other.timestamp - this.timestamp));
                return s;
            }
        }
    }

    private InterpolatingTreeMap<InterpolatingDouble, State> mStateMap;
    private double mDistanceDriven;

    public RobotStateMap()
    {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     * Default value of turretAngle reset is currently zero.
     */
    public synchronized void reset(double startTime, Pose2d initialPose)
    {
        mStateMap = new InterpolatingTreeMap<>(kObservationBufferSize);
        mStateMap.put(new InterpolatingDouble(startTime), new State(initialPose, startTime));
        mDistanceDriven = 0.0;
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     * as well as the turretAngle.
     */
    public synchronized void reset(double startTime, Pose2d initialPose, 
                                    double turretAngle)
    {
        mStateMap = new InterpolatingTreeMap<>(kObservationBufferSize);
        mStateMap.put(new InterpolatingDouble(startTime), 
                      new State(initialPose, turretAngle, startTime));
        mDistanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven()
    {
        mDistanceDriven = 0.0;
    }

    /*
    public synchronized void addObservations(double timestamp, Pose2d pose, 
        Twist2d velI, Twist2d velP)
    {
        InterpolatingDouble ts = new InterpolatingDouble(timestamp);
        mStateMap.put(ts, new State(pose, velI, velP, 0., timestamp));
        mDistanceDriven += velI.dx; // Math.hypot(velocity.dx, velocity.dy);
    }
    */
    
    public synchronized void addObservations(double timestamp, Pose2d pose, 
        Twist2d velI, Twist2d velP, double turretAngle)
    {
        InterpolatingDouble ts = new InterpolatingDouble(timestamp);
        mStateMap.put(ts, new State(pose, velI, velP, turretAngle, timestamp));
        mDistanceDriven += velI.dx; // Math.hypot(velocity.dx, velocity.dy);
        // do we care about time here?
        // no: if dx is measured in distance/loopinterval (loopinterval == 1)
        //
        // do we care about dy here?
        // no: if velocity is in robot coords (no transverse motion expected)
        // yes: if velocity is in field coords

        // The answer to both questions is no, btw.
    }

    /**
     * Returns the robot's state on the field at a certain time. Linearly
     * interpolates between stored robot state to fill in the gaps.
     */
    public synchronized State get(double ts)
    {
        return mStateMap.getInterpolated(new InterpolatingDouble(ts));
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp)
    {
        return this.get(timestamp).pose;
    }

    public synchronized Pose2d getLatestFieldToVehicle()
    {
        return mStateMap.lastEntry().getValue().pose;
    }

    public synchronized State getLatestState()
    {
        return mStateMap.lastEntry().getValue();
    }

    public synchronized double getDistanceDriven()
    {
        return mDistanceDriven;
    }
}
