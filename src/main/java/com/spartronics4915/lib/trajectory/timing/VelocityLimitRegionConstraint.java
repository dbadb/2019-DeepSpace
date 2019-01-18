package com.spartronics4915.lib.trajectory.timing;

import com.spartronics4915.lib.geometry.ITranslation2;
import com.spartronics4915.lib.geometry.Translation2;

public class VelocityLimitRegionConstraint<S extends ITranslation2<S>> implements TimingConstraint<S>
{

    protected final Translation2 min_corner_;
    protected final Translation2 max_corner_;
    protected final double velocity_limit_;

    public VelocityLimitRegionConstraint(Translation2 min_corner, Translation2 max_corner, double velocity_limit)
    {
        min_corner_ = min_corner;
        max_corner_ = max_corner;
        velocity_limit_ = velocity_limit;
    }

    @Override
    public double getMaxVelocity(S state)
    {
        final Translation2 translation = state.getTranslation();
        if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
                translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y())
        {
            return velocity_limit_;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state,
            double velocity)
    {
        return MinMaxAcceleration.kNoLimits;
    }

}
