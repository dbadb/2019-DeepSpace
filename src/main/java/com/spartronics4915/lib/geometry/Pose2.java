package com.spartronics4915.lib.geometry;

import com.spartronics4915.lib.util.Util;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Pose2 implements IPose2<Pose2>
{

    protected static final Pose2 kIdentity = new Pose2();

    public static final Pose2 identity()
    {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2 mTranslation;
    protected final Rotation2 mRotation;

    public Pose2()
    {
        mTranslation = new Translation2();
        mRotation = new Rotation2();
    }

    public Pose2(double x, double y, final Rotation2 rotation)
    {
        mTranslation = new Translation2(x, y);
        mRotation = rotation;
    }

    public Pose2(final Translation2 translation, final Rotation2 rotation)
    {
        mTranslation = translation;
        mRotation = rotation;
    }

    public Pose2(final Pose2 other)
    {
        mTranslation = new Translation2(other.mTranslation);
        mRotation = new Rotation2(other.mRotation);
    }

    public static Pose2 fromTranslation(final Translation2 translation)
    {
        return new Pose2(translation, new Rotation2());
    }

    public static Pose2 fromPoint2(final Point2 pt)
    {
        return new Pose2(pt.toTranslation2(), new Rotation2());
    }

    public static Pose2 fromRotation(final Rotation2 rotation)
    {
        return new Pose2(new Translation2(), rotation);
    }

    /**
     * Obtain a new Pose2 from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     * 
     * See also ch 9 of:
     * http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
     */
    public static Pose2 exp(final Twist2 delta)
    {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps)
        {
            // small angle approximation
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        }
        else
        {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        Translation2 xlate = new Translation2(delta.dx * s - delta.dy * c, 
                                                delta.dx * c + delta.dy * s);
        return new Pose2(xlate,
                new Rotation2(cos_theta, sin_theta, false));
    }

    /**
     * Logical inverse of the above: obtains a Twist2 from a delta-pose
     */
    public static Twist2 log(final Pose2 dPose)
    {
        final double dtheta = dPose.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = dPose.getRotation().cos() - 1.0;
        double halfCos; // halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps)
        {
            halfCos = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halfCos = -(half_dtheta * dPose.getRotation().sin()) / cos_minus_one;
        }
        final Translation2 transPart = dPose.getTranslation()
                .rotateBy(new Rotation2(halfCos, -half_dtheta, false));
        return new Twist2(transPart.x(), transPart.y(), dtheta);
    }

    @Override
    public Translation2 getTranslation()
    {
        return mTranslation;
    }

    public Point2 getPoint2()
    {
        return mTranslation.asPoint2();
    }

    @Override
    public Rotation2 getRotation()
    {
        return mRotation;
    }

    /**
     * Transforming this Pose2 means first translating by
     * other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public Pose2 transformBy(final Pose2 other)
    {
        return new Pose2(mTranslation.translateBy(other.mTranslation.rotateBy(mRotation)),
                mRotation.rotateBy(other.mRotation));
    }

    /**
     * The inverse of this Pose2 "undoes" the effect of applying our transform.
     *
     * For p = new Pose2(10, 10, -30deg)
     *     np = p.inverse()
     * 
     * @return The opposite of this transform.
     */
    public Pose2 inverse()
    {
        Rotation2 invRot = mRotation.inverse();
        return new Pose2(mTranslation.inverse().rotateBy(invRot), invRot);
    }

    public Pose2 normal()
    {
        return new Pose2(mTranslation, mRotation.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of
     * another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2 intersection(final Pose2 other)
    {
        final Rotation2 other_rotation = other.getRotation();
        if (mRotation.isParallel(other_rotation))
        {
            // Lines are parallel.
            return new Translation2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(mRotation.cos()) < Math.abs(other_rotation.cos()))
        {
            return intersectionInternal(this, other);
        }
        else
        {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2 other)
    {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2 twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2 other, double epsilon)
    {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static Translation2 intersectionInternal(final Pose2 a, final Pose2 b)
    {
        final Rotation2 a_r = a.getRotation();
        final Rotation2 b_r = b.getRotation();
        final Translation2 a_t = a.getTranslation();
        final Translation2 b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t))
        {
            return new Translation2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2 interpolate(final Pose2 other, double x)
    {
        if (x <= 0)
        {
            return new Pose2(this);
        }
        else if (x >= 1)
        {
            return new Pose2(other);
        }
        final Twist2 twist = Pose2.log(inverse().transformBy(other));
        return transformBy(Pose2.exp(twist.scaled(x)));
    }

    @Override
    public String toString()
    {
        return "T:" + mTranslation.toString() + ", R:" + mRotation.toString();
    }

    @Override
    public String toCSV()
    {
        return mTranslation.toCSV() + "," + mRotation.toCSV();
    }

    @Override
    public double distance(final Pose2 other)
    {
        return Pose2.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Pose2))
            return false;
        return epsilonEquals((Pose2) other, Util.kEpsilon);
    }

    @Override
    public Pose2 getPose()
    {
        return this;
    }

    @Override
    public Pose2 mirror()
    {
        return new Pose2(new Translation2(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
}
