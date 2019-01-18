package com.spartronics4915.lib.geometry;

/**
 * interface for entities that reside in a Map2
 */
public interface Map2Entry
{
    public class Hit2
    {
        public Point2 p;
        public double dist;
    }

    public Hit2 trace(Ray2 r);
    public Point2 nearestPt(Point2 p);
}