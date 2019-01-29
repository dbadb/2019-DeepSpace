package com.spartronics4915.lib.map;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class Map2Test
{
    public static final double kTestEpsilon = .001;

    @Test
    public void testLine2()
    {
        Line2 a = new Line2(new Point2(-10, 0), new Vec2(1,1)); 

        // nearest test ------------------------------------
        Point2 np = a.nearest(new Point2()); // point on 'a' nearest the origin
        assertNotEquals(np, null);
        assertEquals(np.x, -5, kTestEpsilon);
        assertEquals(np.y, 5, kTestEpsilon);

        // trace tests -------------------------------------
        // intersect a 45 degree line with a vertical ray
        Ray2 r = new Ray2(new Point2(0, 0), new Vec2(0, 1));
        Hit2 h = a.trace(r);
        assertNotEquals(h, null);
        assertEquals(h.hitPt.x, 0.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 10.f, kTestEpsilon);

        // intersect with opposite ray (miss)
        r = new Ray2(new Point2(0, 0), new Vec2(0, -1));
        h = a.trace(r);
        assertEquals(h, null);

        // intersect a with -45 degree ray
        r = new Ray2(new Point2(10, 0), new Vec2(-1,1));
        h = a.trace(r);
        assertNotEquals(h, null);
        assertEquals(h.hitPt.x, 0.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 10.f, kTestEpsilon);

        // contains test ----------------------------------
        //  is currently trivially false for line2 
        assertEquals(a.contains(h.hitPt), false);
    }

    @Test
    public void testSeg2()
    {
        LineSeg2 a = new LineSeg2(new Point2(-1,1), new Point2(1, 1));

        // nearest test ------------------------------------
        Point2 np = a.nearest(new Point2()); // point on 'a' nearest the origin
        assertNotEquals(np, null);
        assertEquals(np.x, 0, kTestEpsilon);
        assertEquals(np.y, 1, kTestEpsilon);

        np = a.nearest(new Point2(-10, 0));
        assertNotEquals(np, null);
        assertEquals(np.x, -1, kTestEpsilon);
        assertEquals(np.y, 1, kTestEpsilon);

        np = a.nearest(new Point2(10, 0));
        assertNotEquals(np, null);
        assertEquals(np.x, 1, kTestEpsilon);
        assertEquals(np.y, 1, kTestEpsilon);

        // trace tests -------------------------------------
        // intersect a horizontal line with a vertical ray
        Ray2 r = new Ray2(new Point2(0, 0), new Vec2(0, 1));
        Hit2 h = a.trace(r);
        assertNotEquals(h, null);
        assertEquals(h.hitPt.x, 0.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 1.f, kTestEpsilon);

        // intersect a vertical ray that misses
        r = new Ray2(new Point2(10, 0), new Vec2(0, 1));
        h = a.trace(r);
        assertEquals(h, null);

        LineSeg2 b = new LineSeg2(new Point2(-1, -1), new Point2(3, 3));
        r = new Ray2(new Point2(4, 0), new Vec2(-1, 1));
        h = b.trace(r);
        assertNotEquals(h, null);
        assertEquals(h.hitPt.x, 2.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 2.f, kTestEpsilon);

        // contains test ----------------------------------
        //  is currently trivially false for sec2
        assertEquals(b.contains(h.hitPt), false);

    }

    @Test
    public void testPolyLine2()
    {
        PolyLine2 r1 = PolyLine2.createRect2(new Point2(-10, 10), 
                                             new Point2(10, -10));

        // nearest test ------------------------------------
        Point2 np = r1.nearest(new Point2(5, 5)); // point on 'a' nearest the origin
        assertNotEquals(np, null);
        assertEquals(np.x, 5, kTestEpsilon);
        assertEquals(np.y, 10, kTestEpsilon);

        // trace tests -------------------------------------
        // intersect a horizontal line with a vertical ray
        Ray2 r = new Ray2(new Point2(0, 0), new Vec2(0, 1));
        Hit2 h = r1.trace(r);
        assertNotEquals(h, null);
        assertEquals(h.hitPt.x, 0.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 10.f, kTestEpsilon);

        // intersect a vertical ray, edge condition
        r = new Ray2(new Point2(10, 0), new Vec2(0, 1));
        h = r1.trace(r);
        assertEquals(h.hitPt.x, 10.f, kTestEpsilon);
        assertEquals(h.hitPt.y, 10.f, kTestEpsilon);

        // vertical ray, edge condition (miss)
        r = new Ray2(new Point2(0, 10), new Vec2(0, 1));
        h = r1.trace(r);
        assertEquals(h, null);

        // contains test ----------------------------------
        assertEquals(r1.contains(new Point2()), true);
        assertEquals(r1.contains(new Point2(40,40)), false);
    }

    @Test
    public void testMap2()
    {
        // Field is centered on origin, 54x27' in inches 
        //      x:[-324,324]
        //      y:[-162,162]
        //  Blue to left, Red to right
        Map2 m = Map2.build2019Map();
        assertEquals(m.contains(new Point2(-1000, -1000)), false);
        assertEquals(m.contains(new Point2(0, 0)), true);

        String mapstr = m.toString();
        System.out.print(mapstr);
        System.out.flush();

        Ray2 ray = new Ray2(new Point2(-200, 0), new Vec2(-1, 0));
        Hit2 h = m.trace(ray);
        assertNotEquals(h, null);
        assertEquals(h.hitEntry.mName, "Blue Hab3");

        ray = new Ray2(new Point2(200, 0), new Vec2(1, 0));
        h = m.trace(ray);
        assertNotEquals(h, null);
        assertEquals(h.hitEntry.mName, "Red Hab3");
    }
}