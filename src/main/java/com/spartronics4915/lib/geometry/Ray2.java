package com.spartronics4915.lib.geometry;

public class Ray2
{
    public final Point2 mOrigin;
    public final Rotation2 mDirection;

    public Ray2(Point2 org, Rotation2 dir)
    {
        mOrigin = org;
        mDirection = dir;
    }
}