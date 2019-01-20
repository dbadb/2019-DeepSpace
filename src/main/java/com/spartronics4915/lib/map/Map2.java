package com.spartronics4915.lib.map;

import java.util.Collection;
import java.util.ArrayList;

// The FRC field is 54'x27' == 648"x324" = 209952 in^2
//  We can choose between a raster or shape-oriented representation.
//  Queries we support:
//      - trace
//      - nearestPt
//      - contains (occupancy test)
//  Notes on shape representation:
//      - small number of shapes required to represent a field
//        4 walls (two side walls might be invisible to trace?)
//        plus field elements.
//      - occupancy test requires awareness of solids.  That plus
//        winding order should be sufficient.
//  Notes on bitmap representation:
//      Occupancy map indicates whether a field location is
//      valid or occupied by a field element. For odometry
//      our accuracy requirements are 2", so we'd need
//      a BitSet with 648/2 * 324/2 = 52488 bits (6561 bytes).
//      The longest distance in the field is 724". Which requires
//      10 bits.  LIDAR maxes out at 12m (~470") and can be represented 
//      in 8 bits at 2" accuracy. If we store 16 range measurements at 
//      each unoccupied field location we require 16*51488 = 823808 bytes (< 1MB).
//      Roborio has 256MB DRAM (plus 512MB nonvolatile). OpenJDK jre binary
//      minimal footprint is 66MB so 1MB map footprint might be viable.

public class Map2 implements Map2Entry
{
    public static final float kFieldWidth = 27 * 12; 
    public static final float kFieldLength = 54 * 12;
    public static final Point2 kFieldCenter = new Point2(kFieldWidth*.5f,
                                                         kFieldLength*.5f);
    public static final PolyLine2 kFieldRect = PolyLine2.createRect(0.f, 0.f,
                                                    kFieldLength, kFieldWidth);

    private ArrayList<Map2Entry> mEntries;

    public Map2(Map2Entry... e)
    {
        this.append(e);
    }

    public Map2(ArrayList<Map2Entry> e)
    {
        this.mEntries = e;
    }

    public void append(Map2Entry... e)
    {
        for(int i=0; i<e.length; i++)
        {
            mEntries.add(e[i]);
        }
    }

    public void append(ArrayList<Map2Entry> ea)
    {
        for(Map2Entry e : ea)
        {
            mEntries.add(e);
        }
    }

    @Override
    public boolean contains(Point2 p)
    {
        boolean ret = true;
        return ret;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        double minDist = Double.MAX_VALUE;
        Hit2 result = null;
        for (Map2Entry e : mEntries) 
        {
            Hit2 hit = e.trace(r);
            if(hit != null && hit.hitDist > 0)
            {
                if (hit.hitDist < minDist) 
                {
                    minDist = hit.hitDist;
                    result = hit;
                }
            }
        }
        return result;
    }

    @Override
    public Point2 nearest(Point2 p)
    {
        double minDist = Double.MAX_VALUE;
        Point2 nearestPoint = null;
        for (Map2Entry e : mEntries) 
        {
            Point2 pp = e.nearest(p);
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