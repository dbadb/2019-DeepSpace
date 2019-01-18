package com.spartronics4915.lib.trajectory.timing;

import com.spartronics4915.lib.geometry.Pose2WithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2WithCurvature>
{

    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel)
    {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2WithCurvature state)
    {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2WithCurvature state, final double velocity)
    {
        return MinMaxAcceleration.kNoLimits;
    }
}
