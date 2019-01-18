package com.spartronics4915.lib.geometry;

public class Point2
{

    public final double x, y;

    public Point2(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Point2(Point2 pt)
    {
        this.x = pt.x;
        this.y = pt.y;
    }

    public Point2(Translation2 t)
    {
        this(t.x(), t.y());
    }

    public Translation2 toTranslation2()
    {
        return new Translation2(x, y);
    }

    public double getDistanceSq(Point2 p)
    {
        double dx = x - p.x, dy = y - p.y;
        return dx * dx + dy * dy;
    }

    public double getDistance(Point2 p)
    {
        return Math.sqrt(getDistanceSq(p));
    }

    public String toString()
    {
        return "(" + x + ", " + y + ")";
    }

}
