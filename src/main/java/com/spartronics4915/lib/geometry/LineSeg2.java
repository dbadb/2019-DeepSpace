package com.spartronics4915.lib.geometry;

public class LineSeg2 implements Map2Entry
{
    public Line2 line;
    public double tMin, tMax;
    public Point2 pMin, pMax;

    public LineSeg2(Line2 line, double tMin, double tMax)
    {
        this.line = line;
        this.tMin = tMin;
        this.tMax = tMax;
        this.pMin = line.getPoint(tMin);
        this.pMax = line.getPoint(tMax);
    }

    public LineSeg2(Point2 p0, Point2 p1)
    {
        this(new Line2(p0, p1), 0, 1);
        normalize();
    }

    @Override
    public Hit2 trace(Ray2 ray)
    {
        Hit2 ret = null;
        return ret;
    }

    @Override
    public Point2 nearestPt(Point2 p)
    {
        return this.getClosestPoint(p);
    }

    @Override
    public boolean contains(Point2 p)
    {
        return false; 
    }

    private void normalize()
    {
        double mSq = line.vx * line.vx + line.vy * line.vy;
        double m = Math.sqrt(mSq);
        line.vx /= m;
        line.vy /= m;
        line.r /= m;
        tMax *= m;
    }

    public double getDistance(Point2 p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin.getDistance(p);
        if (t >= tMax)
            return pMax.getDistance(p);
        return line.getDistance(p);
    }

    public double getDistanceSq(Point2 p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin.getDistanceSq(p);
        if (t >= tMax)
            return pMax.getDistanceSq(p);
        double d = line.getDistance(p);
        return d * d;
    }

    public Point2 getClosestPoint(Point2 p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin;
        if (t >= tMax)
            return pMax;
        return line.getPoint(t);
    }

    public Point2 getMidpoint()
    {
        return line.getPoint((tMin + tMax) / 2);
    }

    public String toString()
    {
        return "LineSeg2(" + line + ", [" + tMin + ", " + tMax + "])";
    }

}
