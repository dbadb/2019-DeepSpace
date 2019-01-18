package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Point2;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashSet;

/**
 * Holds a single 360 degree scan from the lidar.  The timestamp
 * for the scan is that of the first point.
 */
public class LidarScan 
{
    private LidarSample[] mLidarSamples;
    private Point2[] mPoints;
    private double mTimestamp;
    private int mIndex;
    private static final int k_maxScanSize = 400;

    public LidarScan()
    {
        mTimestamp = 0;
        mLidarSamples = new LidarSample[k_maxScanSize];
        mPoints = new Point2[k_maxScanSize];
        mIndex = 0;
    }

    public LidarSample[] getSamples()
    {
        return mLidarSamples;
    }

    public Point2[] getPoints()
    {
        return mPoints;
    }

    public int getScanSize()
    {
        return mIndex;
    }

    public double getTimestamp()
    {
        return mTimestamp;
    }

    public void addSample(LidarSample s, Pose2 robotPose)
    {
        if (mTimestamp == 0)
            mTimestamp = s.timestamp;

        if(mIndex < k_maxScanSize)
        {
            mLidarSamples[mIndex] = s;
            mPoints[mIndex] = s.toCartesian(robotPose);
            mIndex++;
        }
    }

    /**
     * @param resolution: maximum resolution in inches.
     * @return list of points, culled according to resolution.
     */
    public ArrayList<Point2> getCulledPoints(double resolution)
    {
        ArrayList<Point2> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (int i=0;i<mIndex;i++)
        {
            Point2 p = mPoints[i];
            if (buckets.add(getBucket(p.x, p.y, resolution)))
                list.add(p);
        }
        return list;
    }

    public String toJsonString() 
    {
        String json = "{\"class\":\"lidarscan\"" + 
                    ", \"timestamp\": " + mTimestamp + 
                    ", \"pt2list\": [";
        NumberFormat fmt = NumberFormat.getInstance();
        for (int i=0;i<mIndex;i++)
        {
            // json += "{\"x\":" + point.x + ", \"y\":" + point.y + "},";
            Point2 p = mPoints[i];
            if(i > 0) json += ",";
            json += String.format("[%.3f,%.3f]", p.x, p.y);
        }
        json += "]}";
        return json;
    }

    public String toString()
    {
        String s = "";
        for (int i=0;i<mIndex;i++)
        {
            Point2 p = mPoints[i];
            s += "x: " + p.x + ", y: " + p.y + "\n";
        }
        return s;
    }


    /**
     * Cantor pairing function (to bucket & hash two doubles)
     * converts two integers into one; used as a hash key
     */
    private int getBucket(double x, double y, double bucketSize)
    {
        int ix = (int) (x / bucketSize);
        int iy = (int) (y / bucketSize);
        int a = ix >= 0 ? 2 * ix : -2 * ix - 1;
        int b = iy >= 0 ? 2 * iy : -2 * iy - 1;
        int sum = a + b;
        return sum * (sum + 1) / 2 + a;
    }

}
