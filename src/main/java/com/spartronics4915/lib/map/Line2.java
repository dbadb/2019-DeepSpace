package com.spartronics4915.lib.map;

public class Line2 implements Map2Entry
{
    public Point2 origin; // infinite line intersects this point
    public Vec2 direction; // unit length

    public Line2()
    {
        origin = new Point2(0,0);
        direction = new Vec2(1,0);
    }

    public Line2(Point2 o, Vec2 dir)
    {
        origin = o;
        direction = dir;
        direction.normalize();
    }

    /**
     * @param t - a distance from the line's 'origin'
     * @return a point on the line at distance t
     */
    public Point2 getPointOnLine(float t)
    {
        return new Point2(origin.x + t * direction.x,
                          origin.y + t * direction.y);
    }

    /**
     * @param p presumed to be on this line, often obtained via intersection
     * @return the parametric value for a point presumed to be on this line
     *  if the point isn't on the line the value 0 is returned.
     */
    public float getT(Point2 p)
    {
        Vec2 diff = p.subtract(origin);
        Vec2 dir = Vec2.normalize(diff);
        if(Math.abs(dir.dot(this.direction)-1) < .001)
        {
            return (float) Math.hypot(diff.x, diff.y);
        }
        else
            return 0;
    }

    @Override
    public Point2 nearest(Point2 p)
    {
        Vec2 diff = p.subtract(this.origin);
        float t = this.direction.dot(diff);
        Point2 closest = new Point2(this.origin);
        closest.x += t * this.direction.x;
        closest.y += t * this.direction.y;
        return closest;
    }

    @Override
    public Hit2 trace(Ray2 r)
    {
        // The intersection of two lines is a solution to 
        //      P0 + s0*D0 = P1 + s1*D1.
        // Rewrite this as s0*D0 - s1*D1 = P1 - P0 = diff 
        // (here r is P0, this is P1)
        // If DotPerp(D0, D1)) = 0, the lines are parallel.  
        //      If DotPerp(diff, D1)) = 0, the lines are the same.  
        // else
        // If Dotperp(D0, D1)) is not zero, then    
        //   s0 = DotPerp(diff, D1))/DotPerp(D0, D1))    
        // produces the point of intersection.  Also, 
        //   s1 = DotPerp(diff, D0))/DotPerp(D0, D1))
        Hit2 ret = null;
        float D0D1 = r.direction.dotperp(this.direction);
        if(D0D1 != 0.f)
        {
            /* bingo, lines not parallel/coincident */
            Vec2 diff = this.origin.subtract(r.origin); // diff not normalized
            float diffD1 = diff.dotperp(this.direction); // diffDotPerpD1
            float invD0D1 = 1.f / D0D1;
            float s0 = diffD1 * invD0D1;
            if(s0 > 0)
            {
                ret = new Hit2();
                ret.ray = r;
                ret.rayT = s0;
                ret.hitPt = new Point2(r.origin.x + s0 * r.direction.x,
                                       r.origin.y + s0 * r.direction.y);
                ret.hitT = diff.dotperp(r.direction) * invD0D1;
            }
        }
        else
        {
            /* XXX: distinguish between parallel and colinear */
        }
        return ret;
    }

    @Override
    public boolean contains(Point2 p)
    {
        return false; // though we could consider half-space
    }
}