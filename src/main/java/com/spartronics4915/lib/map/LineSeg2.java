package com.spartronics4915.lib.map;

public class LineSeg2 implements Map2Entry
{
    public Point2 p0, p1;

    public LineSeg2(Point2 p0, Point2 p1)
    {
        this.p0 = p0;
        this.p1 = p1;
    }

    public LineSeg2(Line2 line, float tMin, float tMax)
    {
        this.p0 = line.getPointOnLine(tMin);
        this.p1 = line.getPointOnLine(tMax);
    }

    @Override
    public Hit2 trace(Ray2 ray)
    {
        Line2 l2 = new Line2();
        float extent = this.getCenteredLine(l2);
        Hit2 ret = l2.trace(ray);
        if(ret != null)
        {
            if(Math.abs(ret.hitT) > extent)
                ret = null;
        }
        return ret;
    }

    @Override
    public Point2 nearest(Point2 p)
    {
        Vec2 dir = this.p1.subtract(this.p0); // not normalized yet
        Vec2 diff = p.subtract(this.p1);
        float t = dir.dot(diff);
        Point2 segClosest;
        if(t >= 0)
        {
            segClosest = this.p1; // t == 1
        }
        else
        {
            diff = p.subtract(this.p0);
            t = dir.dot(diff);
            if(t <= 0.f)
            {
                segClosest = this.p0; // t == 0
            }
            else
            {
                // the interesting case
                float sqr = dir.dot(dir);
                if(sqr > 0) // t interesting
                {
                    t /= sqr;
                    segClosest = new Point2(this.p0.x + t * dir.x,
                                            this.p0.y + t * dir.y);
                }
                else
                {
                    segClosest = this.p0; // t == 0
                }
            }
        }
        return segClosest;
    }

    @Override
    public boolean contains(Point2 p)
    {
        return false;  // currently trivially false
    }

    public Point2 getMidpoint()
    {
        return new Point2(.5f * (this.p0.x + this.p1.x),
                          .5f * (this.p0.y + this.p1.y));
    }

    public String toString()
    {
        return "LineSeg2(" + this.p0 + "," + this.p1 + ")";
    }

    /**
     * 
     * @param l2 - converts LineSeg2 to a Line2 whose origin is our midpt
     * @return - half segment length, so t should vary from [-hlen, hlen]
     */
    private float getCenteredLine(Line2 l2)
    {
        l2.origin = new Point2(.5f*(this.p0.x + this.p1.x),
                            .5f*(this.p0.y + this.p1.y));
        l2.direction = this.p1.subtract(this.p0);
        return .5f * l2.direction.normalize();
    }


}
