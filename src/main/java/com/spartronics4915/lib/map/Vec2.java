package com.spartronics4915.lib.map;

/**
 * can be used to represent a point or a direction.
 */
public class Vec2
{
    public float x, y;

    public Vec2()
    {
        this.x = 0.f;
        this.y = 0.f;
    }
    
    public Vec2(Vec2 a)
    {
        this.x = a.x;
        this.y = a.y;
    }

    public Vec2(float x, float y)
    {
        this.x = x;
        this.y = y;
    }

    public Vec2(double x, double y)
    {
        this.x = (float) x;
        this.y = (float) y;
    }

    public float normalize()
    {
        float len = (float) Math.hypot(this.x, this.y);
        if(len > 0.f)
        {
            this.x /= len;
            this.y /= len;
        }
        return len;
    }

    public static Vec2 normalize(Vec2 a)
    {
        Vec2 b = new Vec2(a);
        b.normalize();
        return b;
    }

    public float length()
    {
        return (float) Math.sqrt(this.lengthSq());
    }

    public float lengthSq()
    {
        return dot(this);
    }

    public float dot(Vec2 o)
    {
        return this.x*o.x + this.y*o.y;
    }
    
    /**
     * @return Vec2 that's perpendicular to this.
     *   When viewed from origin toward direction, 
     *   the perp is on the right side
     * 
     *      vec(1, 0)  -> vec(0, -1)
     *          ---->  ->    |
     *                       v
     *      vec(-1, 0) -> vec(0, 1)
     *          <----  ->    ^
     *                       |
     *      vec(0, 1)  -> vec(1, 0)
     *          ^  
     *          |      ->  --->
     * 
     *      vec(0, -1) -> vec(-1, 0)
     *          |  
     *          v      ->  <---
     *          
     */
    public Vec2 perp()
    {
        return new Vec2(this.y, -this.x);
    }

    /**
     * @param o
     * @return the cos of the angle between this and o's perpendicular
     */
    public float dotperp(Vec2 o)
    {
        return this.dot(o.perp());
    }

    public String toString()
    {
        return "Vec2(" + this.x + "," + this.y + ")";
    }
}