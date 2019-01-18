package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.geometry.Point2;
import com.spartronics4915.lib.geometry.Transform2;
import java.util.ArrayList;

public class PointCloudReferenceModel implements IReferenceModel
{
    private Iterable<Point2> mPointCloud;

    public PointCloudReferenceModel(Iterable<Point2> pointCloud)
    {
        mPointCloud = pointCloud;
    }

    @Override
    public Point2 getClosestPoint(Point2 refPnt)
    {
        double minDist = Double.MAX_VALUE;
        Point2 minPnt = null;
        for (Point2 testPnt : mPointCloud)
        {
            double dist = refPnt.getDistanceSq(testPnt);
            if (dist < minDist)
            {
                minPnt = testPnt;
                minDist = dist;
            }
        }
        return minPnt;
    }

    @Override
    public void transformBy(Transform2 t)
    {
        ArrayList<Point2> transformedPoints = new ArrayList<Point2>();
        for (Point2 p : mPointCloud)
        {
            transformedPoints.add(t.apply(p));
        }
        mPointCloud = transformedPoints;
    }

}
