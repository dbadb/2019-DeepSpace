package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.geometry.Point2;
import com.spartronics4915.lib.geometry.Pose2;

/**
 * Represents a single point from the LIDAR sensor. This consists of
 * an angle, distance, and timestamp.
 */
class LidarSample
{
    public static final double MM_TO_IN = 1 / 25.4; // 1 inch = 25.4 millimeters

    public final double timestamp;
    public final double angle;
    public final double distance;

    public LidarSample(double timestamp, double angle, double distance)
    {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance * MM_TO_IN;
    }

    /**
     * Convert this point into a {@link Translation2} in cartesian (x, y)
     * coordinates. The point's timestamp is used along with the {@link RobotStateMap}
     * to take into account the robot's pose at the time the point was detected.
     * 
     * @param robotPose: optional robotPose, used to transform lidar points
     *  to field coordinates.  If not provided, we're operating in "relative"
     *  mode.  If provided, robotPose should include the robotToLidar 
     *  transform.
     */
    public Point2 toCartesian(Pose2 robotPose)
    {
        // convert the polar coords to cartesian coords
        double radians = Math.toRadians(this.angle);
        Point2 x2d = new Point2(Math.cos(radians) * this.distance, 
                                  Math.sin(radians) * this.distance);
        if(robotPose != null)
        {
            return robotPose.transformBy(
                    Pose2.fromPoint2(x2d)).
                        getTranslation().asPoint2();
        }
        else
            return x2d;
    }
}
