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

    public Point2 relativePoint(float dx, float dy)
    {
        return new Point2(this.x + dx, this.y + dy);
    }

    public void translate(Vec2 dx)
    {
        this.x += dx.x;
        this.y += dx.y;
    }

    public Vec2 subtract(Point2 p2)
    {
        return new Vec2(this.x - p2.x, this.y - p2.y);
    }

    public float distanceSq(Point2 p2)
    {
        float dx = p2.x - this.x;
        float dy = p2.y - this.y;
        return  dx*dx + dy*dy;
    }

    public float distance(Point2 p2)
    {
        return (float) Math.sqrt(this.distanceSq(p2));
    }

    public String toString()
    {
        return "Point2(" + this.x + "," + this.y + ")";
    }

}