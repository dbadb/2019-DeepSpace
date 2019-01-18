package com.spartronics4915.lib.geometry;

import com.spartronics4915.lib.util.Util;

import java.text.DecimalFormat;

public class Pose2WithCurvature implements IPose2<Pose2WithCurvature>, ICurvature<Pose2WithCurvature>
{

    protected static final Pose2WithCurvature kIdentity = new Pose2WithCurvature();

    public static final Pose2WithCurvature identity()
    {
        return kIdentity;
    }

    protected final Pose2 pose_;
    protected final double curvature_;
    protected final double dcurvature_ds_;

    public Pose2WithCurvature()
    {
        pose_ = new Pose2();
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public Pose2WithCurvature(final Pose2 pose, double curvature)
    {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2WithCurvature(final Pose2 pose, double curvature, double dcurvature_ds)
    {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2WithCurvature(final Translation2 translation, final Rotation2 rotation, double curvature)
    {
        pose_ = new Pose2(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2WithCurvature(final Translation2 translation, final Rotation2 rotation, double curvature, double dcurvature_ds)
    {
        pose_ = new Pose2(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    @Override
    public final Pose2 getPose()
    {
        return pose_;
    }

    @Override
    public Pose2WithCurvature transformBy(Pose2 transform)
    {
        return new Pose2WithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    @Override
    public Pose2WithCurvature mirror()
    {
        return new Pose2WithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public double getCurvature()
    {
        return curvature_;
    }

    @Override
    public double getDCurvatureDs()
    {
        return dcurvature_ds_;
    }

    @Override
    public final Translation2 getTranslation()
    {
        return getPose().getTranslation();
    }

    @Override
    public final Rotation2 getRotation()
    {
        return getPose().getRotation();
    }

    @Override
    public Pose2WithCurvature interpolate(final Pose2WithCurvature other, double x)
    {
        return new Pose2WithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final Pose2WithCurvature other)
    {
        return getPose().distance(other.getPose());
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Pose2WithCurvature))
            return false;
        Pose2WithCurvature p2dwc = (Pose2WithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature())
                && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    @Override
    public String toCSV()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," + fmt.format(getCurvature()) + "," + fmt.format(getDCurvatureDs());
    }
}
