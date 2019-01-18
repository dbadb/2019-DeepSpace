package com.spartronics4915.lib.spline;

import com.spartronics4915.lib.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class SplineGenerator
{

    private static final double kMaxDX = 2.0; //inches
    private static final double kMaxDY = 0.05; //inches
    private static final double kMaxDTheta = 0.1; //radians!
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2WithCurvature that approximates the original spline
     */
    public static List<Pose2WithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double maxDTheta, double t0, double t1)
    {
        List<Pose2WithCurvature> rv = new ArrayList<>();
        rv.add(s.getPose2WithCurvature(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize)
        {
            getSegmentArc(s, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    /**
     * Convenience function to parametrize a spline from t 0 to 1
     */
    public static List<Pose2WithCurvature> parameterizeSpline(Spline s)
    {
        return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }

    public static List<Pose2WithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double maxDTheta)
    {
        return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
    }

    public static List<Pose2WithCurvature> parameterizeSplines(List<Spline> splines)
    {
        return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }

    public static List<Pose2WithCurvature> parameterizeSplines(List<? extends Spline> splines, double maxDx, double maxDy,
            double maxDTheta)
    {
        List<Pose2WithCurvature> rv = new ArrayList<>();
        if (splines.isEmpty())
            return rv;
        rv.add(splines.get(0).getPose2WithCurvature(0.0));
        for (final Spline s : splines)
        {
            List<Pose2WithCurvature> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(Spline s, List<Pose2WithCurvature> rv, double t0, double t1, double maxDx,
            double maxDy,
            double maxDTheta)
    {
        Translation2 p0 = s.getPoint(t0);
        Translation2 p1 = s.getPoint(t1);
        Rotation2 r0 = s.getHeading(t0);
        Rotation2 r1 = s.getHeading(t1);
        Pose2 transformation = new Pose2(new Translation2(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        Twist2 twist = Pose2.log(transformation);
        if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
        {
            // subdivide
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        }
        else
        {
            rv.add(s.getPose2WithCurvature(t1));
        }
    }

}
