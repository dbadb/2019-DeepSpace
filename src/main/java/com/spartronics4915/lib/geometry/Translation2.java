package com.spartronics4915.lib.geometry;

import com.spartronics4915.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an
 * (x, y) plane.
 */
public class Translation2 implements ITranslation2<Translation2>
{

    protected static final Translation2 kIdentity = new Translation2();

    public static final Translation2 identity()
    {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public Translation2()
    {
        x_ = 0;
        y_ = 0;
    }

    public Translation2(double x, double y)
    {
        x_ = x;
        y_ = y;
    }

    public Translation2(final Translation2 other)
    {
        x_ = other.x_;
        y_ = other.y_;
    }

    public Translation2(final Translation2 start, final Translation2 end)
    {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm()
    {
        return Math.hypot(x_, y_);
    }

    public double norm2()
    {
        return x_ * x_ + y_ * y_;
    }

    public double x()
    {
        return x_;
    }

    public double y()
    {
        return y_;
    }

    public Point2 asPoint2()
    {
        return new Point2(x_, y_);
    }

    /**
     * We can compose Translation2's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2 translateBy(final Translation2 other)
    {
        return new Translation2(x_ + other.x_, y_ + other.y_);
    }

    public Translation2 translateBy(final double x, final double y)
    {
        return new Translation2(x_ + x, y_ + y);
    }

    /**
     * We can also rotate Translation2's. See:
     * https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2 rotateBy(final Rotation2 rotation)
    {
        return new Translation2(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public Rotation2 direction()
    {
        return new Rotation2(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2 that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2 inverse()
    {
        return new Translation2(-x_, -y_);
    }

    @Override
    public Translation2 interpolate(final Translation2 other, double x)
    {
        if (x <= 0)
        {
            return new Translation2(this);
        }
        else if (x >= 1)
        {
            return new Translation2(other);
        }
        return extrapolate(other, x);
    }

    public Translation2 extrapolate(final Translation2 other, double x)
    {
        return new Translation2(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public Translation2 scale(double s)
    {
        return new Translation2(x_ * s, y_ * s);
    }

    public boolean epsilonEquals(final Translation2 other, double epsilon)
    {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }

    @Override
    public String toCSV()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x_) + "," + fmt.format(y_);
    }

    public static double dot(final Translation2 a, final Translation2 b)
    {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    public static Rotation2 getAngle(final Translation2 a, final Translation2 b)
    {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle))
        {
            return new Rotation2();
        }
        return Rotation2.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(final Translation2 a, final Translation2 b)
    {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }

    @Override
    public double distance(final Translation2 other)
    {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Translation2))
            return false;
        return distance((Translation2) other) < Util.kEpsilon;
    }

    @Override
    public Translation2 getTranslation()
    {
        return this;
    }
}
