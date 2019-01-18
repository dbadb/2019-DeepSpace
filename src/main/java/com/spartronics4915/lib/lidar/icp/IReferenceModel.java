package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.geometry.Point2;
import com.spartronics4915.lib.geometry.Transform2;

public interface IReferenceModel
{
    public Point2 getClosestPoint(Point2 p);
    public void transformBy(Transform2 t);
}
