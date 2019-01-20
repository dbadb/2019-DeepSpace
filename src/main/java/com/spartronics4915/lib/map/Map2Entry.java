package com.spartronics4915.lib.map;

/**
 * interface for entities that reside in a Map2
 */
public interface Map2Entry
{
    /**
     * Trace the given ray against the geometry shape (or shape-group)
     * @param r - the ray to trace
     * @return - a Hit2, null if no intersection if found
     */
    public Hit2 trace(Ray2 r);

    /**
     *  Find the nearest point on the shape (or shape-group)
     * @param p
     * @return
     */
    public Point2 nearest(Point2 p);

    /**
     * 
     * @param p
     * @return true if p is contained by shape (or shape-group)
     */
    public boolean contains(Point2 p);
}