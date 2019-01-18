package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.lidar.LidarScan;
import com.spartronics4915.lib.geometry.Point2;
import com.spartronics4915.lib.geometry.Transform2;

public class RelativeICPProcessor
{
    private final ICP mICP;
    private final Transform2 mZero;
    private IReferenceModel mLastReferenceModel;

    /**
     * Instantiate a RelativeICPProcessor and have it make its own ICP object. You
     * must specify the convergence timeout for the ICP object that will be made.
     * 
     * @param icpTimeoutMs Convergence timeout for ICP
     */
    public RelativeICPProcessor(long icpTimeoutMs)
    {
        this(new ICP(icpTimeoutMs));
    }

    /**
     * Instantiate a RelativeICPProcessor with a caller-supplied ICP object (e.g.
     * you alread have one made, and don't want to instantiate a whole new one)
     * 
     * @param icp Premade ICP object
     */
    public RelativeICPProcessor(ICP icp)
    {
        mICP = icp;
        mZero = new Transform2();
    }

    /**
     * Applies ICP point registration, using the last provided point cloud as a
     * reference. Returns a Transform that can be used to register the old
     * point cloud to the new one. This is a representation of the relative
     * motion of the robot. Since we never convert to field coordinates, we 
     * don't care about absolute robot pose or even vehicleToLidar
     * 
     * @param pointCloud
     * @return The relative transform to transform first pointcloud to second.
     */
    public Transform2 doRelativeICP(LidarScan scan)
    {
        Transform2 result;
        if(mLastReferenceModel != null)
            result = mICP.doICP(scan, mZero, mLastReferenceModel);
        else
            result = new Transform2(); // ie no-tranform
        mLastReferenceModel = new PointCloudReferenceModel(
                                    scan.getCulledPoints(3/*in res*/));
        return result;
    }
}
