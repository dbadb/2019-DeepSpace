package com.spartronics4915.lib.map;

public class Point2 extends Vec2
{
    public Point2()
    {
        super();
    }

    public Point2(Point2 p)
    {
        super(p.x, p.y);
    }

    public Point2(float x, float y)
    {
        super(x, y);
    }

    public Vec2 subtract(Point2 p2)
    {
        return new Vec2(this.x - p2.x, this.y - p2.y);
    }

    public String toString()
    {
        return "Point2(" + this.x + "," + this.y + ")";
    }

}