package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Twist2;

public interface IPathFollower
{

    public Twist2 steer(Pose2 current_pose);

    public boolean isDone();
}
