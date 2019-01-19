package com.spartronics4915.lib.map;

/**
 * can be used to represent a point or a direction.
 */
public class Vec2
{
    public float x, y;

    public Vec2()
    {
        this.x = 0F;
        this.y = 0F;
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


}