package com.spartronics4915.lib.geometry;

import java.util.Collection;

// The FRC field is 54'x27' == 648"x324" = 209952 in^2
//  Occupancy map indicates whether a field location is
//  valid or occupied by a field element. For odometry
//  our accuracy requirements are 2", so we'd need
//  a BitSet with 648/2 * 324/2 = 52488 bits (6561 bytes).
//  The longest distance in the field is 724". Which requires
//  10 bits.  LIDAR maxes out at 12m (~470") and can be represented 
//  in 8 bits at 2" accuracy. If we store 16 range measurements at 
//  each unoccupied field location we require 16*51488 = 823808 bytes (< 1MB).
//  Roborio has 256MB DRAM (plus 512MB nonvolatile). OpenJDK jre binary
//  minimal footprint is 66MB so 1MB map footprint might be viable.
// 
//  Since space is at a premium we must take care with object overhead
//  associated with Boxed primitive types.  Multidimensional arrays also
//  include per-row overhead. 
//   https://stackoverflow.com/questions/258120/what-is-the-memory-consumption-of-an-object-in-java
//

public class Map2 implements Map2Entry
{
    public static final double kFieldWidth = 27 * 12; 
    public static final double kFieldLength = 54 * 12;
    public static final Point2 kFieldCenter = new Point2(kFieldWidth*.5,
                                                            kFieldLength*.5);
    public static final Rect2 kFieldRect = new Rect2(0, 0, kFieldWidth, kFieldLength);

    private final Map2Entry[] mEntries;
    public Map2(Map2Entry... e)
    {
        if(e.length == 0)
            throw new IllegalArgumentException("zero Segments passed to ReferenceModel");
        mEntries = e;
    }

    public Map2(Collection<Map2Entry> ee)
    {
        this(ee.toArray(new Map2Entry[ee.size()]));
    }

    public boolean occupied(int x, int y)
    {
        boolean ret = true;
        return ret;
    }

    public Hit2 trace(Ray2 r)
    {
        double minDist = Double.MAX_VALUE;
        Hit2 result = null;
        for (Map2Entry e : mEntries) 
        {
            Hit2 hit = e.trace(r);
            if(hit != null && hit.dist > 0)
            {
                if (hit.dist < minDist) 
                {
                    minDist = hit.dist;
                    result = hit;
                }
            }
        }
        return result;
    }

    public Point2 nearestPt(Point2 p)
    {
        double minDist = Double.MAX_VALUE;
        Point2 nearestPoint = null;
        for (Map2Entry e : mEntries) 
        {
            Point2 pp = e.nearestPt(p);
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