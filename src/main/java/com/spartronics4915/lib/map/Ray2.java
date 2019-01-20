package com.spartronics4915.lib.map;

/**
 * a Ray2 is merely a Line2 which excludes negative distances
 */
public class Ray2 extends Line2
{
    public Ray2(Point2 org, Vec2 dir)
    {
        super(org, dir);
    }

    // todo: implement trace as call to super then check for hitT < 0

    public String toString()
    {
        return "Ray2(" + this.origin + "," + this.direction + ")";
    }
}