package com.spartronics4915.lib.geometry;

/**
 * interface for entities that reside in a Map2
 */
public interface Map2Entry
{
    public class Hit2
    {
        public Ray2 ray;
        public double hitDist;
        public Point2 hitPt;
    }

    public Hit2 trace(Ray2 r);
    public Point2 nearestPt(Point2 p);
    public boolean contains(Point2 p);
}