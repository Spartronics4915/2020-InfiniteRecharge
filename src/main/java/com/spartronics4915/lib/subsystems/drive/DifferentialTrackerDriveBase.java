package com.spartronics4915.lib.subsystems.drive;

import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker.TrajectoryTrackerOutput;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive.ChassisState;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive.DriveDynamics;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive.WheelState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

/**
 * This interface gives you a little more help than
 * {@link TrajectoryTrackerDriveBase}. It makes it so that you don't have to use
 * a SpartronicsSubsystem, but also so that you don't need to deal with all the
 * kinematics/dynamics stuff yourself.
 */
public interface DifferentialTrackerDriveBase extends TrajectoryTrackerDriveBase
{
    public DifferentialDrive getDifferentialDrive();

    @Override
    public default void setOutput(TrajectoryTrackerOutput outputMeters)
    {
        setOutputFromDynamics(outputMeters.getDifferentialDriveVelocity(),
            outputMeters.getDifferentialDriveAcceleration());
    }

    public default void setOutputFromKinematics(ChassisState chassisVelocityMetersPerSecond)
    {
        WheelState wheelVelocities = getDifferentialDrive()
            .solveInverseKinematics(chassisVelocityMetersPerSecond);
        WheelState feedForwardVoltages = getDifferentialDrive().getVoltagesFromkV(wheelVelocities);

        setOutput(wheelVelocities, feedForwardVoltages);
    }

    public default void setOutputFromDynamics(ChassisState chassisVelocityMetersPerSecond,
        ChassisState chassisAccelerationMetersPerSecondSq)
    {
        DriveDynamics dynamics = getDifferentialDrive().solveInverseDynamics(
            chassisVelocityMetersPerSecond, chassisAccelerationMetersPerSecondSq);

        setOutput(dynamics.wheelVelocity, dynamics.voltage);
    }

    public default void setOutput(WheelState wheelVelocitiesRadiansPerSecond,
        WheelState wheelVoltages)
    {
        SmartDashboard.putNumber("Drive/leftSpeedTarget", wheelVelocitiesRadiansPerSecond.left * Units.inchesToMeters(8));
        SmartDashboard.putNumber("Drive/rightSpeedTarget", wheelVelocitiesRadiansPerSecond.right  * Units.inchesToMeters(8));

        getLeftMotor().setVelocity(
            (wheelVelocitiesRadiansPerSecond.left * getDifferentialDrive().wheelRadius()),
            wheelVoltages.left);
        getRightMotor().setVelocity(
            (wheelVelocitiesRadiansPerSecond.right * getDifferentialDrive().wheelRadius()),
            wheelVoltages.right);
    }
}
