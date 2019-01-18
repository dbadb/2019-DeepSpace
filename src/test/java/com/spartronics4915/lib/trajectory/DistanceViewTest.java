package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DistanceViewTest
{

    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test()
    {
        // Specify desired waypoints.
        List<Translation2> waypoints = Arrays.asList(
                new Translation2(0.0, 0.0),
                new Translation2(24.0, 0.0),
                new Translation2(36.0, 0.0),
                new Translation2(36.0, 24.0),
                new Translation2(60.0, 24.0));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2> trajectory = new Trajectory<>(waypoints);
        final DistanceView<Translation2> distance_view = new DistanceView<>(trajectory);

        assertEquals(0.0, distance_view.first_interpolant(), kTestEpsilon);
        assertEquals(84.0, distance_view.last_interpolant(), kTestEpsilon);

        assertEquals(waypoints.get(0), distance_view.sample(0.0).state());
        assertEquals(waypoints.get(0).interpolate(waypoints.get(1), 0.5), distance_view.sample(12.0).state());
        assertEquals(waypoints.get(3).interpolate(waypoints.get(4), 0.5), distance_view.sample(72.0).state());
    }

}
