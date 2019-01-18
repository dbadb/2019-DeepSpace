package com.spartronics4915.lib.geometry;

// everything you want to know about ellipse2
//  currently unimplemented.
//
// http://mathworld.wolfram.com/Ellipse-LineIntersection.html
// https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
// https://www.geometrictools.com/GTEngine/Include/Mathematics/GteDistPointHyperellipsoid.h

class Ellipse2 implements Map2Entry
{
    private Point2 mOrigin;
    private double mRadiusX, mRadiusY; // to rotate we need a transform

    public Ellipse2(Point2 org, double radX, double radY)
    {
        mOrigin = new Point2(org);
        mRadiusX = radX;
        mRadiusY = radY;
    }

    @Override
    public Hit2 trace(Ray2 ray)
    {
        Hit2 ret;
        return ret;
    }

    @Override
    public Point2 nearestPt(Point2 p)
    {
        Point2 ret;
        return ret;
    }
}