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

public class Map2 extends Map2Entry
{
    private ArrayList<Map2Entry> mEntries;

    /**
     * 
     * @param x coordinate of center of rocket
     * @param y coordinate of back of rocket
     * @param flip top-side rockets are flipped, bottom side, not
     * @return closed polyline representing plan view of rocket.
     */
    /*
                    18.5
            c________________d                  _
            /                \                  |
           /                  \                 |
          /                    \                |
         /                      \               | 27.44
       b/                        \e             |
        |                        |  7.53        |
        a-----------0------------f              _
                   38.7 
    */
    static PolyLine2 build2019Rocket(float x, float y, boolean flipY)
    {
        PolyLine2 ret = new PolyLine2();
        float abY = 7.53f;
        float bcX = 38.7f - .5f * 18.5f;
        float bcY = 27.44f - 7.53f;
        if(flipY)
        {
            abY = -abY;
            bcY = -bcY;
            ret.setOrientation(PolyLine2.Orientation.kReversed);
        }
        else
            ret.setOrientation(PolyLine2.Orientation.kStandard);
        Point2 a = new Point2(x-.5f*38.7f, 0);
        Point2 b = a.relativePoint(0, abY);
        Point2 c = b.relativePoint(bcX, bcY);
        Point2 d = c.relativePoint(18.5f, 0);
        Point2 e = d.relativePoint(bcX, -bcY);
        Point2 f = d.relativePoint(0, -abY);

        ret.append(new LineSeg2(a, b));
        ret.append(new LineSeg2(b, c));
        ret.append(new LineSeg2(c, d));
        ret.append(new LineSeg2(d, e));
        ret.append(new LineSeg2(e, f));
        ret.append(new LineSeg2(f, a));

        return ret;
    }

    static Map2 build2019Map()
    {
        // Field origin is center, x along length, y along width (z is height)
        // L is Blue, R is Red
        Map2 ret = new Map2();
        final float kFieldWidth = 27 * 12; 
        final float kFieldLength = 54 * 12;
        final float kFieldHalfX = kFieldLength * .5f;
        final float kFieldHalfY = kFieldWidth * .5f;
        final PolyLine2 kFieldRect = PolyLine2.createRect2(
                                        new Point2(-kFieldHalfX, kFieldHalfY),
                                        new Point2(kFieldHalfX, -kFieldHalfY));
        kFieldRect.mName = "Field2019";
        ret.append(kFieldRect);

        final PolyLine2 kRocket1 = build2019Rocket(96, kFieldHalfY, true); // Red, North
        kRocket1.mName = "Red North Rocket";
        final PolyLine2 kRocket2 = build2019Rocket(-96, kFieldHalfY, true); // Blue, North
        kRocket2.mName = "Blue North Rocket";
        final PolyLine2 kRocket3 = build2019Rocket(96, -kFieldHalfY, false); // Red, South
        kRocket3.mName = "Red South Rocket";
        final PolyLine2 kRocket4 = build2019Rocket(-96, -kFieldHalfY, false); // Blue, South
        kRocket3.mName = "Blue South Rocket";
        ret.append(kRocket1);
        ret.append(kRocket2);
        ret.append(kRocket3);
        ret.append(kRocket4);

        // HAB -------------------------------------------------------------
        // level three hab platforms (1 per side) (48x48x3)
        final PolyLine2 kHab3L = PolyLine2.createRect2(new Point2(0, 0),
                                                       new Point2(48, 48));
        kHab3L.translate(new Vec2(-kFieldHalfX, -24));
        kHab3L.mName = "Blue Hab3";
        ret.append(kHab3L);

        final PolyLine2 kHab3R = PolyLine2.createRect2(new Point2(0, 0),
                                                        new Point2(48, 48));
        kHab3R.translate(new Vec2(kFieldHalfX-48, -24));
        kHab3R.mName = "Red Hab3";
        ret.append(kHab3R);

        // level two hab platforms (2 per side)j 48x40x1 (6" higher than ramp)
        final PolyLine2 kHab2LT = PolyLine2.createRect2(new Point2(0, 0),
                                                        new Point2(48, 40));
        kHab2LT.translate(new Vec2(-kFieldHalfX, 24 + 40)); 
        kHab2LT.mName = "Blue North Hab2";
        ret.append(kHab2LT);

        final PolyLine2 kHab2LB = PolyLine2.createRect2(new Point2(0, 0),
                                                        new Point2(48, 40));
        kHab2LB.translate(new Vec2(-kFieldHalfX, -24)); 
        kHab2LB.mName = "Blue South Hab2";
        ret.append(kHab2LB);

        final PolyLine2 kHab2RT = PolyLine2.createRect2(new Point2(0, 0),
                                                        new Point2(48, 40));
        kHab2RT.translate(new Vec2(kFieldHalfX-48, 24 + 40)); 
        kHab2RT.mName = "Red North Hab2";
        ret.append(kHab2RT);

        final PolyLine2 kHab2RB = PolyLine2.createRect2(new Point2(0, 0),
                                                        new Point2(48, 40));
        kHab2RB.translate(new Vec2(kFieldHalfX-48, -24)); 
        kHab2RB.mName = "Red South Hab2";
        ret.append(kHab2RB);

        // Cargo Ships
        // Each Cargo Ship is a 7'11.75" long by 4'7.75"width, 4" tall
        // Centered on  midline (Y) and 9" from center line (x)
        // This is the floor footprint, higher up the profile is more curvy.
        final PolyLine2 kShipL = PolyLine2.createRect2(new Point2(0, 0),
                                                    new Point2(95.75f, 55.75f));
        final PolyLine2 kShipR = PolyLine2.createRect2(new Point2(0, 0),
                                                    new Point2(95.75f, 55.75f));
        kShipL.translate(new Vec2(-95.75f - 9, .5f*55.75f));
        kShipL.mName = "Blue Cargo Ship";
        ret.append(kShipL);
        kShipR.translate(new Vec2(9, .5f*55.75f));
        kShipR.mName = "Red Cargo Ship";
        ret.append(kShipR);

        return ret;
    }

    public Map2(ArrayList<Map2Entry> e)
    {
        this.mEntries = e;
    }

    public Map2()
    {
        mEntries = new ArrayList<>();
    }

    public Map2(Map2Entry... e)
    {
        this();
        this.append(e);
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
    public String toString()
    {
        String ret = "<map2\n";
        if(mName != null)
            ret += "  id=\"" + mName + "\"\n";
        
        for (Map2Entry e : mEntries) 
            ret += e.toString();
        ret += ">\n";
        return ret;
    }

    @Override
    public void translate(Vec2 dx)
    {
        for (Map2Entry e : mEntries) 
        {
            e.translate(dx);
        }
    }

    @Override
    public boolean contains(Point2 p)
    {
        // currently a maps contains a point if any of its components
        // contain it.
        for (Map2Entry e : mEntries) 
        {
            if(e.contains(p))
                return true;
        }
        return false;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        float minDist = Float.MAX_VALUE;
        Hit2 result = null;
        for (Map2Entry e : mEntries) 
        {
            Hit2 hit = e.trace(r);
            if(hit != null)
            {
                float dist = hit.distance();
                if (dist > 0 && dist < minDist) 
                {
                    minDist = dist;
                    result = hit;
                }
            }
        }
        return result;
    }

    @Override
    public Point2 nearest(Point2 p)
    {
        float minDist = Float.MAX_VALUE;
        Point2 nearestPoint = null;
        for (Map2Entry e : mEntries) 
        {
            Point2 pp = e.nearest(p);
            float distSq = pp.distanceSq(p);
            if (distSq < minDist) 
            {
                minDist = distSq;
                nearestPoint = pp;
            }
        }
        return nearestPoint;
    }
}