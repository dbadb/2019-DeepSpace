package com.spartronics4915.frc2019;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Twist2;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * skidding).
 */

public class Kinematics
{

    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate
     * than below, but useful for predicting motion)
     */
    public static Twist2 forwardKinematics(double left_wheel_delta, double right_wheel_delta)
    {
        double delta_rotation = (right_wheel_delta - left_wheel_delta)
                / (Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    public static Twist2 forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads)
    {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2(dx, 0.0, delta_rotation_rads);
    }

    public static Twist2 forwardKinematics(Rotation2 prev_heading, double left_wheel_delta, double right_wheel_delta,
            Rotation2 current_heading)
    {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist2(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2 and previous
     * rotation.
     */
    public static Pose2 integrateForwardKinematics(Pose2 current_pose, Twist2 forward_kinematics)
    {
        return current_pose.transformBy(Pose2.exp(forward_kinematics));
    }
}
