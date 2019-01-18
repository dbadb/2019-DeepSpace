package com.spartronics4915.lib.spline;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Pose2WithCurvature;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;

public abstract class Spline
{

    public abstract Translation2 getPoint(double t);

    public abstract Rotation2 getHeading(double t);

    public abstract double getCurvature(double t);

    // dk/ds
    public abstract double getDCurvature(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    public Pose2 getPose2(double t)
    {
        return new Pose2(getPoint(t), getHeading(t));
    }

    public Pose2WithCurvature getPose2WithCurvature(double t)
    {
        return new Pose2WithCurvature(getPose2(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
    }

    // TODO add toString
    // public abstract String toString();
}
