package com.spartronics4915.lib.spline;

import com.spartronics4915.lib.geometry.*;
import com.spartronics4915.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SplineGeneratorTest
{

    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test()
    {
        // Create the test spline
        Pose2 p1 = new Pose2(new Translation2(0, 0), new Rotation2());
        Pose2 p2 = new Pose2(new Translation2(15, 10), new Rotation2(1, -5, true));
        Spline s = new QuinticHermiteSpline(p1, p2);

        List<Pose2WithCurvature> samples = SplineGenerator.parameterizeSpline(s);

        double arclength = 0;
        Pose2WithCurvature cur_pose = samples.get(0);
        for (Pose2WithCurvature sample : samples)
        {
            final Twist2 t = Pose2.log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
            arclength += t.dx;
            cur_pose = sample;
        }

        assertEquals(cur_pose.getTranslation().x(), 15.0, kTestEpsilon);
        assertEquals(cur_pose.getTranslation().y(), 10.0, kTestEpsilon);
        assertEquals(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
        assertEquals(arclength, 23.17291953186379, kTestEpsilon);
    }
}
