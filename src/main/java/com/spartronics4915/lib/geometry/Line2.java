package com.spartronics4915.lib.geometry;

import java.util.Arrays;
import java.util.Collection;

public class Line2
{
    public static Line2 fitLine2(Collection<Point2> points)
    {
        int n = points.size();

        double xMean = 0, yMean = 0;
        for (Point2 p : points)
        {
            xMean += p.x;
            yMean += p.y;
        }
        xMean /= n;
        yMean /= n;

        double sxx = 0, sxy = 0, syy = 0;
        for (Point2 p : points)
        {
            double dx = p.x - xMean;
            double dy = p.y - yMean;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        sxx /= n - 1;
        sxy /= n - 1;
        syy /= n - 1;

        double dsxy = syy - sxx;
        double vy = dsxy + Math.sqrt(dsxy * dsxy + 4 * sxy * sxy);
        double vx = 2 * sxy;
        double mag = Math.hypot(vx, vy);
        double r = (vy * xMean - vx * yMean) / mag;
        return new Line2(vx, vy, r);
    }

    public double vx, vy, r; // vy*x - vx*y = r
    public double x0, y0;

    public Line2(double vx, double vy, double r)
    {
        this.vx = vx;
        this.vy = vy;
        this.r = r;

        x0 = vy * r;
        y0 = -vx * r;
    }

    public Line2(double vx, double vy, double x0, double y0)
    {
        this.vx = vx;
        this.vy = vy;
        this.x0 = x0;
        this.y0 = y0;

        r = vy * x0 - vx * y0;
    }

    public Line2(Point2 p0, Point2 p1)
    {
        this(p1.x - p0.x, p1.y - p0.y, p0.x, p0.y);
    }

    public double getDistance(Point2 p)
    {
        return Math.abs(vy * p.x - vx * p.y - r);
    }

    public LineSeg2 getSegment(Collection<Point2> points)
    {
        double minT = Double.MAX_VALUE, maxT = -Double.MAX_VALUE;
        for (Point2 p : points)
        {
            double t = getT(p.x, p.y);
            if (t < minT)
                minT = t;
            if (t > maxT)
                maxT = t;
        }
        return new LineSeg2(this, minT, maxT);
    }

    public double getT(double x, double y)
    {
        return vx * (x - x0) + vy * (y - y0);
    }

    public double getT(Point2 p)
    {
        return getT(p.x, p.y);
    }

    public Point2 getPoint(double t)
    {
        return new Point2(x0 + vx * t, y0 + vy * t);
    }

    public String toString()
    {
        return Arrays.toString(new double[] {vx, vy, r, x0, y0});
    }

}
