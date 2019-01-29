package com.spartronics4915.lib.map;

public class Hit2
{
    public Ray2 ray;
    public float rayT=0; /* of ray */
    public Map2Entry hitEntry;
    public Point2 hitPt;
    public Vec2 hitPerp;
    public float hitT; /* of hit obj */
    private float hitDist=-1f;
    public float distance()
    {
        if(rayT > 0)
        {
            if(hitDist <= 0)
                hitDist = ray.origin.distance(hitPt);
            return hitDist;
        }
        else
            return 0;
    }
    public float distanceSq()
    {
        if(rayT > 0)
            return ray.origin.distanceSq(hitPt);
        else
            return 0;
    }
}
