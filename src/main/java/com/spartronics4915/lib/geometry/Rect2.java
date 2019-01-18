package com.spartronics4915.lib.geometry;

public class Rect2 implements Map2Entry
{
    private LineSeg2[] mSegments;

    public Rect2(Point2 cornerOne, Point2 cornerTwo)
    {
        // These may not be top or right if cornerTwo is left or higher than
        // cornerOne but it's easier to reason about this by naming things this way.
        Point2 topRight = new Point2(cornerTwo.x, cornerOne.y);
        Point2 bottomLeft = new Point2(cornerOne.x, cornerTwo.y);
        this.mSegments = new LineSeg2[] {
                            new LineSeg2(cornerOne, topRight),
                            new LineSeg2(topRight, cornerTwo),
                            new LineSeg2(cornerTwo, bottomLeft),
                            new LineSeg2(bottomLeft, topRight),
                        };
    }

    public Rect2(double x0, double y0, double x1, double y1)
    {
        this(new Point2(x0, y0), new Point2(x1, y1));
    }

    public boolean ptInRect(Point2 p)
    {
        return false;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        Hit2 ret;
        return ret;
    }

    @Override
    public Point2 nearestPt(Point2 p)
    {
        Point2 ret;
        return ret;
    }

}