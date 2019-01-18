package com.spartronics4915.lib.geometry;

public class Transform2
{
    // rotate by theta about the origin, then translate by <tx, ty>
    public final double theta, tx, ty;
    protected final double sin, cos; // cache these

    public Transform2()
    {
        theta = 0;
        tx = ty = 0;
        sin = 0.0;
        cos = 1.0;
    }

    public Transform2(double theta, double tx, double ty)
    {
        this(theta, tx, ty, Math.sin(theta), Math.cos(theta));
    }

    public Transform2(double theta, double tx, double ty, double sin, double cos)
    {
        this.theta = theta;
        this.tx = tx;
        this.ty = ty;
        this.sin = sin;
        this.cos = cos;
    }

    public Transform2(Pose2 pose)
    {
        Translation2 t = pose.getTranslation();
        Rotation2 r = pose.getRotation();
        theta = r.getRadians();
        sin = r.sin();
        cos = r.cos();
        tx = t.x();
        ty = t.y();
    }

    public Pose2 toPose2()
    {
        return new Pose2(new Translation2(tx, ty), new Rotation2(cos, sin, false));
    }

    public Point2 apply(Point2 p)
    {
        return new Point2(p.x * cos - p.y * sin + tx,
                p.x * sin + p.y * cos + ty);
    }

    public Line2 apply(Line2 l)
    {
        return new Line2(l.vx * cos - l.vy * sin,
                l.vx * sin + l.vy * cos,
                l.x0 * cos - l.y0 * sin + tx,
                l.x0 * sin + l.y0 * cos + ty);
    }

    public LineSeg2 apply(LineSeg2 s)
    {
        return new LineSeg2(apply(s.line), s.tMin, s.tMax);
    }

    public Transform2 inverse()
    {
        return new Transform2(-theta,
                -tx * cos - ty * sin,
                tx * sin - ty * cos,
                -sin, cos);
    }

    public String toString()
    {
        return "[" + Math.toDegrees(theta) + "° <" + tx + ", " + ty + ">]";
    }

}
