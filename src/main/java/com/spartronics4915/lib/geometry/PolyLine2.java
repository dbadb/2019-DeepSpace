package com.spartronics4915.lib.geometry;

import java.util.ArrayList;

public class PolyLine2 implements Map2Entry
{
    public enum Occupancy
    {
        kUndefined,
        kAsSolid, // presumes closed 
        kAsHole   // ditto
    }

    private ArrayList<LineSeg2> mSegments;
    private Occupancy mOccupancy;

    static PolyLine2 createRect2(Point2 cornerOne, Point2 cornerTwo)
    {
        // These may not be top or right if cornerTwo is left or higher than
        // cornerOne but it's easier to reason about this by naming things this way.
        Point2 topRight = new Point2(cornerTwo.x, cornerOne.y);
        Point2 bottomLeft = new Point2(cornerOne.x, cornerTwo.y);
        return new PolyLine2(new LineSeg2(cornerOne, topRight),
                            new LineSeg2(topRight, cornerTwo),
                            new LineSeg2(cornerTwo, bottomLeft),
                            new LineSeg2(bottomLeft, topRight));
    }

    static PolyLine2 createRect(double x0, double y0, double x1, double y1)
    {
        return createRect2(new Point2(x0, y0), new Point2(x1, y1));
    }

    public PolyLine2(ArrayList<LineSeg2> al)
    {
        mSegments = al;
        mOccupancy = Occupancy.kUndefined;
    }

    public PolyLine2(LineSeg2... al)
    {
        mSegments = new ArrayList<LineSeg2>();
        mOccupancy = Occupancy.kUndefined;
        for(int i=0; i<e.length; i++)
        {
            mSegments.add(al[i]);
        }
    }

    public void setOccupancy(Occupancy o)
    {
        mOccupancy = o;
    }

    public Occupancy getOccupancy()
    {
        return mOccupancy;
    }

    @Override
    public boolean contains(Point2 p)
    {
        // Find nearestPt, then dot-product vector with line.
        //  * if we are known to be convex, then any point woud suffice
        boolean ret = false;
        double minDist = Double.MAX_VALUE;
        LineSeg2 minLine = null;
        for (LineSeg2 ls : mSegments) 
        {
            Point2 pp = ls.nearestPt(p);
            double distSq = pp.getDistanceSq(p);
            if (distSq < minDist) 
            {
                minDist = distSq;
                minLine = ls;
            }
        }
        if(minLine != null)
        {
            if(ls.)
        }
        return ret;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        Hit2 ret = null;
        for(LineSeg2 ls : mSegments)
        {

        }
        return ret;
    }

    @Override
    public Point2 nearestPt(Point2 p)
    {
        double minDist = Double.MAX_VALUE;
        Point2 nearestPoint = null;
        for (LineSeg2 ls : mSegments) 
        {
            Point2 pp = ls.nearestPt(p);
            double distSq = pp.getDistanceSq(p);
            if (distSq < minDist) 
            {
                minDist = distSq;
                nearestPoint = pp;
            }
        }
        return nearestPoint;
    }

}