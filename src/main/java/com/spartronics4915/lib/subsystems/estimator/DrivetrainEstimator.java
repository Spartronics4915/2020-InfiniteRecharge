package com.spartronics4915.lib.subsystems.estimator;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Twist2d;

import edu.wpi.first.wpilibj.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N6;

public class DrivetrainEstimator
{

    private static final double kNominalDt = 0.01;

    private final ExtendedKalmanFilter<N3, N3, N3> mObserver;
    private final ExtendedKalmanFilter<N3, N3, N6> mVisionObserver;

    private final Matrix<N3, N1> f(Matrix<N3, N1> x, Matrix<N3, N1> u)
    {
        // Diff drive forward kinematics:
        // v_c = (v_l + v_r) / 2
        var newPose = new Twist2d((u.get(0, 0) + u.get(1, 0)) / 2, 0.0,
            Rotation2d.fromRadians(u.get(2, 0))).exp().transformBy(
                new Pose2d(x.get(0, 0), x.get(1, 0), Rotation2d.fromRadians(x.get(2, 0))));

        return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(newPose.getTranslation().getX(),
            newPose.getTranslation().getY(), x.get(2, 0) + u.get(2, 0));
    }

    private final Matrix<N6, N1> h(Matrix<N3, N1> x, Matrix<N3, N1> u)
    {
        return new MatBuilder<>(Nat.N6(), Nat.N1()).fill(x.get(0, 0), x.get(1, 0), x.get(2, 0),
            x.get(0, 0), x.get(1, 0), x.get(2, 0));
    }

    public DrivetrainEstimator(Matrix<N3, N1> stateStdDevs, Matrix<N6, N1> measurementStdDevs)
    {
        mObserver = new ExtendedKalmanFilter<N3, N3, N3>(Nat.N3(), Nat.N3(), Nat.N3(), this::f,
            (x, u) -> x, stateStdDevs,
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(measurementStdDevs.get(0, 0),
                measurementStdDevs.get(1, 0), measurementStdDevs.get(2, 0)),
            false, kNominalDt);
        mVisionObserver = new ExtendedKalmanFilter<>(Nat.N3(), Nat.N3(), Nat.N6(), this::f, this::h,
            stateStdDevs, measurementStdDevs, false, kNominalDt);
    }

    public Pose2d update(Pose2d slamRobotPose, Pose2d visionRobotPose, double dleftMeters,
        double drightMeters, double dthetaRadians)
    {
        var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(dleftMeters, drightMeters, dthetaRadians);

        if (visionRobotPose != null)
        {
            mVisionObserver.setP(mObserver.getP());
            mVisionObserver.setXhat(mObserver.getXhat());

            var y = new MatBuilder<>(Nat.N6(), Nat.N1()).fill(slamRobotPose.getTranslation().getX(),
                slamRobotPose.getTranslation().getY(), slamRobotPose.getRotation().getRadians(),
                visionRobotPose.getTranslation().getX(), visionRobotPose.getTranslation().getY(),
                visionRobotPose.getRotation().getRadians());

            mVisionObserver.correct(u, y);
            mVisionObserver.predict(u, kNominalDt);

            mObserver.setP(mVisionObserver.getP());
            mObserver.setXhat(mVisionObserver.getXhat());
        }
        else
        {
            var y = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(slamRobotPose.getTranslation().getX(),
                slamRobotPose.getTranslation().getY(), slamRobotPose.getRotation().getRadians());

            mObserver.correct(u, y);
            mObserver.predict(u, kNominalDt);
        }

        return new Pose2d(mObserver.getXhat(0), mObserver.getXhat(1),
            Rotation2d.fromRadians(mObserver.getXhat(2)));
    }

}
