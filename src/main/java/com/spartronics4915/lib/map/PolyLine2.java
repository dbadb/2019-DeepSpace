package com.spartronics4915.lib.map;

import java.util.ArrayList;

public class PolyLine2 extends Map2Entry
{
    public enum Orientation
    {
        kUndefined,
        kStandard,
        kReversed
    }

    private ArrayList<LineSeg2> mSegments;
    private Orientation mOrientation;

    static PolyLine2 createRect2(Point2 cornerOne, Point2 cornerTwo)
    {
        // These may not be top or right if cornerTwo is left or higher than
        // cornerOne but it's easier to reason about this by naming things this way.
        // We want unique copies of all points to ensure independent editing.
        Point2 topRight = new Point2(cornerTwo.x, cornerOne.y);
        Point2 bottomLeft = new Point2(cornerOne.x, cornerTwo.y);
        PolyLine2 ret = new PolyLine2(
                            new LineSeg2(cornerOne, topRight),
                            new LineSeg2(new Point2(topRight), cornerTwo),
                            new LineSeg2(new Point2(cornerTwo), bottomLeft),
                            new LineSeg2(new Point2(bottomLeft), 
                                         new Point2(cornerOne))
                            );
        // this winding area produces normals to the Inside
        ret.mOrientation = Orientation.kStandard;
        return ret; 
    }

    static PolyLine2 createRect(float x0, float y0, float x1, float y1)
    {
        return createRect2(new Point2(x0, y0), new Point2(x1, y1));
    }

    static PolyLine2 createRectC(Point2 center, float xsize, float ysize)
    {
        return createRect(center.x - xsize/2f, center.y - ysize/2f,
                           center.x + xsize/2f, center.x + ysize/2f);
    }

    public PolyLine2(ArrayList<LineSeg2> al)
    {
        mSegments = al;
        mOrientation = Orientation.kUndefined;
    }

    public PolyLine2(LineSeg2... al)
    {
        mSegments = new ArrayList<LineSeg2>();
        mOrientation = Orientation.kUndefined;
        for(int i=0; i<al.length; i++)
        {
            mSegments.add(al[i]);
        }
    }

    public void append(LineSeg2 ls)
    {
        mSegments.add(ls);
    }

    public void reverseOrientation()
    {
        switch(mOrientation)
        {
        case kStandard:
            mOrientation = Orientation.kReversed;
            break;
        case kReversed:
            mOrientation = Orientation.kStandard;
            break;
        default:
            break;
        }
    }

    public void setOrientation(Orientation o)
    {
        mOrientation = o;
    }

    public Orientation getOrientation()
    {
        return mOrientation;
    }

    @Override
    public String toString()
    {
        String ret = "<polyline\n";
        if(mName != null)
            ret += "  id=\"" + mName + "\"\n";
        
        for (Map2Entry e : mSegments) 
            ret += e.toString();
        ret += ">\n";
        return ret;
    }

    @Override
    public void translate(Vec2 dx)
    {
        for (LineSeg2 ls : mSegments) 
        {
            ls.translate(dx);
        }
    }

    @Override
    public boolean contains(Point2 p)
    {
        boolean ret = false;
        if(mOrientation != Orientation.kUndefined)
        {
            // Trace in arbitrary direction. Dot hitPerp with ray.
            // Here we're using winding order to determine inside-outside.
            // 
            Ray2 ray = new Ray2(p, new Vec2(1, 0));
            Hit2 hit = this.trace(ray);
            if(hit != null)
            {
                // N is on the right of the vector.
                // If the pline is clockwise, a point is inside
                // If orientation is reversed
                // if ray.N < 0.
                //           |
                //           |
                //     <--N--|
                //           | 
                // -ray----->|   line dir(0, 1), perp(1, -0);
                //           |
                //           v
                //
                float dot = ray.direction.dot(hit.hitPerp);
                if(dot <= 0) // hit from inside
                    ret = true;
                if(mOrientation == Orientation.kReversed)
                    ret = !ret;
            }
        }
        return ret;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        Hit2 ret = null;
        float minDist = Float.MAX_VALUE;
        for(LineSeg2 ls : mSegments)
        {
            Hit2 lh = ls.trace(r);
            if(lh != null)
            {
                float ld = lh.distanceSq();
                if(ld < minDist)
                {
                    minDist = ld;
                    ret = lh;
                }
            }
        }
        if(ret != null)
            ret.hitEntry = this;
        return ret;
    }

    @Override
    public Point2 nearest(Point2 p)
    {
        float minDist = Float.MAX_VALUE;
        Point2 nearestPoint = null;
        for (LineSeg2 ls : mSegments) 
        {
            Point2 pp = ls.nearest(p);
            float distSq = p.subtract(pp).lengthSq();
            if (distSq < minDist) 
            {
                minDist = distSq;
                nearestPoint = pp;
            }
        }
        return nearestPoint;
    }

}