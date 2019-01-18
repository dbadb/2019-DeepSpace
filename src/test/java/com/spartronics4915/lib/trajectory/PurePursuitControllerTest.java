package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.geometry.Twist2;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class PurePursuitControllerTest
{

    @Test
    public void test()
    {
        List<Translation2> waypoints = Arrays.asList(
                new Translation2(0.0, 0.0),
                new Translation2(24.0, 0.0),
                new Translation2(36.0, 12.0),
                new Translation2(60.0, 12.0));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2> reference_trajectory = new Trajectory<>(waypoints);
        DistanceView<Translation2> arc_length_parameterized_trajectory = new DistanceView<>(reference_trajectory);
        PurePursuitController<Translation2> controller = new PurePursuitController<>(
                arc_length_parameterized_trajectory, 1.0, 6.0, 0.1);

        Pose2 robot_pose = new Pose2(waypoints.get(0), Rotation2.identity());
        final int kMaxIter = 100;
        int i = 0;
        for (; i < kMaxIter; ++i)
        {
            if (controller.isDone())
                break;
            Twist2 steering_command = controller.steer(robot_pose);
            steering_command = steering_command.scaled(1.0 / Math.max(1.0, steering_command.norm()));
            System.out.println("Iter: " + i + ", Pose: " + robot_pose + ", Steering Command: " + steering_command);
            robot_pose = robot_pose.transformBy(Pose2.exp(steering_command));
        }
        assertTrue(i < kMaxIter);
    }

}
