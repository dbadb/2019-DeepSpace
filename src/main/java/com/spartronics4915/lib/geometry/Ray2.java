package com.spartronics4915.lib.geometry;

public class Ray2
{
    public Translation2 origin;
    public double[] dir = new double[2];
    public double[] invdir = new double[2];

    public class Intersection2d
    {
        public double tMin, tMax;
        public double[] norm = new double[2];
        // could also include "material" or object id
    }
}