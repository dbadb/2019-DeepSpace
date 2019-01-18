package com.spartronics4915.lib.trajectory.timing;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.util.Util;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TimedStateTest
{

    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test()
    {
        // At (0,0,0), t=0, v=0, acceleration=1
        final TimedState<Pose2> start_state = new TimedState<>(Pose2.fromTranslation(new Translation2(0.0, 0.0)),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        final TimedState<Pose2> end_state = new TimedState<>(Pose2.fromTranslation(new Translation2(0.5, 0.0)), 1.0,
                1.0, 0.0);

        assertEquals(start_state, start_state.interpolate(end_state, 0.0));
        assertEquals(end_state, start_state.interpolate(end_state, 1.0));
        assertEquals(end_state, end_state.interpolate(start_state, 0.0));
        System.out.println(end_state.interpolate(start_state, 1.0));
        assertEquals(start_state, end_state.interpolate(start_state, 1.0));

        final TimedState<Pose2> intermediate_state = start_state.interpolate(end_state, 0.5);
        assertEquals(0.5, intermediate_state.t(), kTestEpsilon);
        assertEquals(start_state.acceleration(), intermediate_state.acceleration(), kTestEpsilon);
        assertEquals(0.5, intermediate_state.velocity(), kTestEpsilon);
        assertEquals(0.125, intermediate_state.state().getTranslation().x(), kTestEpsilon);
    }

}
