package com.spartronics4915.lib.geometry;

import com.spartronics4915.lib.util.Util;

import java.text.DecimalFormat;

import static com.spartronics4915.lib.util.Util.kEpsilon;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Rotation2 implements IRotation2<Rotation2>
{

    protected static final Rotation2 kIdentity = new Rotation2();

    public static final Rotation2 identity()
    {
        return kIdentity;
    }

    protected final double cos_angle_;
    protected final double sin_angle_;

    public Rotation2()
    {
        this(1, 0, false);
    }

    public Rotation2(double x, double y, boolean normalize)
    {
        if (normalize)
        {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon)
            {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            }
            else
            {
                sin_angle_ = 0;
                cos_angle_ = 1;
            }
        }
        else
        {
            cos_angle_ = x;
            sin_angle_ = y;
        }
    }

    public Rotation2(final Rotation2 other)
    {
        cos_angle_ = other.cos_angle_;
        sin_angle_ = other.sin_angle_;
    }

    public Rotation2(final Translation2 direction, boolean normalize)
    {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation2 fromRadians(double angle_radians)
    {
        return new Rotation2(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    public static Rotation2 fromDegrees(double angle_degrees)
    {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public double cos()
    {
        return cos_angle_;
    }

    public double sin()
    {
        return sin_angle_;
    }

    public double tan()
    {
        if (Math.abs(cos_angle_) < kEpsilon)
        {
            if (sin_angle_ >= 0.0)
            {
                return Double.POSITIVE_INFINITY;
            }
            else
            {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle_ / cos_angle_;
    }

    public double getRadians()
    {
        return Math.atan2(sin_angle_, cos_angle_);
    }

    public double getDegrees()
    {
        return Math.toDegrees(getRadians());
    }

    /**
     * We can rotate this Rotation2 by adding together the effects of it and
     * another rotation.
     *
     * @param other The other rotation. See:
     *              https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2 rotateBy(final Rotation2 other)
    {
        return new Rotation2(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
    }

    public Rotation2 normal() /* ie: perpendicular */
    {
        return new Rotation2(-sin_angle_, cos_angle_, false);
    }

    /**
     * The inverse of a Rotation2 "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation2 inverse()
    {
        return new Rotation2(cos_angle_, -sin_angle_, false);
    }

    public boolean isParallel(final Rotation2 other)
    {
        return Util.epsilonEquals(Translation2.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation2 toTranslation()
    {
        return new Translation2(cos_angle_, sin_angle_);
    }

    @Override
    public Rotation2 interpolate(final Rotation2 other, double x)
    {
        if (x <= 0)
        {
            return new Rotation2(this);
        }
        else if (x >= 1)
        {
            return new Rotation2(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2.fromRadians(angle_diff * x));
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }

    @Override
    public String toCSV()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(getDegrees());
    }

    @Override
    public double distance(final Rotation2 other)
    {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Rotation2))
            return false;
        return distance((Rotation2) other) < Util.kEpsilon;
    }

    @Override
    public Rotation2 getRotation()
    {
        return this;
    }
}
