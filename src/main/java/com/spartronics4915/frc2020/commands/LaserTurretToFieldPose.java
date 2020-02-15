package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaserTurretToFieldPose extends CommandBase
{
    private final TalonSRX mTurretMotor;
    private final RobotStateMap mStateMap;

    private final Pose2d kTarget = new Pose2d(0, Units.inchesToMeters(13.75), new Rotation2d());
    private final Pose2d kRobotToTurret = new Pose2d(-Units.inchesToMeters(3), 0, new Rotation2d());
    private final Rotation2d kTurretZeroOffset = Rotation2d.fromDegrees(100);

    public LaserTurretToFieldPose(RobotStateMap stateMap)
    {
        mTurretMotor = new TalonSRX(7);
        mTurretMotor.configFactoryDefault();
        mTurretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        mTurretMotor.configAllowableClosedloopError(0, 40);
        mTurretMotor.config_kP(0, 5);
        mTurretMotor.config_kI(0, 0);
        mTurretMotor.config_kD(0, 1000);
        mTurretMotor.config_kF(0, 0);

        mStateMap = stateMap;
    }

    @Override
    public void execute()
    {
        var visionUpdate = SmartDashboard.getNumberArray("Vision/test", new double[] {0, 0, 0});
        var fieldToRobot = new Pose2d(visionUpdate[2], visionUpdate[0], mStateMap.getLatestFieldToVehicle().getRotation()).transformBy(Constants.Estimator.kVisionToRobot);//mStateMap.getLatestFieldToVehicle();
        var fieldToTurret = fieldToRobot.transformBy(kRobotToTurret);
        var targetToTurret = fieldToTurret.inFrameReferenceOf(kTarget).getTranslation();
        var fieldAnglePointingToTarget = new Rotation2d(targetToTurret.getTranslation().getX(), targetToTurret.getTranslation().getY(), true).inverse();
        var angle = fieldAnglePointingToTarget.rotateBy(fieldToTurret.getRotation());
        setAngle(angle);
    }

    @Override
    public Set<Subsystem> getRequirements()
    {
        return Set.of();
    }

    private void setAngle(Rotation2d angle) {
        SmartDashboard.putNumber("Drive/leftSpeedTarget", toNativeUnits(angle.rotateBy(kTurretZeroOffset)));
        SmartDashboard.putNumber("acutal", mTurretMotor.getSelectedSensorPosition());
        mTurretMotor.set(ControlMode.Position, toNativeUnits(angle.rotateBy(kTurretZeroOffset)));
    }

    private double toNativeUnits(Rotation2d angle) {
        return (angle.getDegrees() / 360.0) * 1024 * 10.0;
    }
}