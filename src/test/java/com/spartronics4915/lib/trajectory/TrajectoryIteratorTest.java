package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class TrajectoryIteratorTest
{

    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2> kWaypoints = Arrays.asList(
            new Translation2(0.0, 0.0),
            new Translation2(24.0, 0.0),
            new Translation2(36.0, 12.0),
            new Translation2(60.0, 12.0));

    @Test
    public void test()
    {
        Trajectory<Translation2> traj = new Trajectory<>(kWaypoints);
        TrajectoryIterator<Translation2> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertEquals(kWaypoints.get(0), iterator.getState());
        assertFalse(iterator.isDone());

        // Advance forward.
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.preview(0.5).state());
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.advance(0.5).state());
        assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance backwards.
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.preview(-0.25).state());
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.advance(-0.25).state());
        assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance past end.
        assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        assertEquals(kWaypoints.get(3), iterator.advance(5.0).state());
        assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.isDone());

        // Advance past beginning.
        assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        assertEquals(kWaypoints.get(0), iterator.advance(-5.0).state());
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());
    }

}
