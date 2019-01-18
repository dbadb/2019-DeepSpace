package com.spartronics4915.lib.spline;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class QuinticHermiteOptimizerTest
{

    private static double kEpsilon = Util.kEpsilon;

    @Test
    public void test()
    {
        Pose2 a = new Pose2(new Translation2(0, 100), Rotation2.fromDegrees(270));
        Pose2 b = new Pose2(new Translation2(50, 0), Rotation2.fromDegrees(0));
        Pose2 c = new Pose2(new Translation2(100, 100), Rotation2.fromDegrees(90));

        List<QuinticHermiteSpline> splines = new ArrayList<>();
        splines.add(new QuinticHermiteSpline(a, b));
        splines.add(new QuinticHermiteSpline(b, c));

        long startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines) < 0.014);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2 d = new Pose2(new Translation2(0, 0), Rotation2.fromDegrees(90));
        Pose2 e = new Pose2(new Translation2(0, 50), Rotation2.fromDegrees(0));
        Pose2 f = new Pose2(new Translation2(100, 0), Rotation2.fromDegrees(90));
        Pose2 g = new Pose2(new Translation2(100, 100), Rotation2.fromDegrees(0));

        List<QuinticHermiteSpline> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermiteSpline(d, e));
        splines1.add(new QuinticHermiteSpline(e, f));
        splines1.add(new QuinticHermiteSpline(f, g));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines1) < 0.16);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2 h = new Pose2(new Translation2(0, 0), Rotation2.fromDegrees(0));
        Pose2 i = new Pose2(new Translation2(50, 0), Rotation2.fromDegrees(0));
        Pose2 j = new Pose2(new Translation2(100, 50), Rotation2.fromDegrees(45));
        Pose2 k = new Pose2(new Translation2(150, 0), Rotation2.fromDegrees(270));
        Pose2 l = new Pose2(new Translation2(150, -50), Rotation2.fromDegrees(270));

        List<QuinticHermiteSpline> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermiteSpline(h, i));
        splines2.add(new QuinticHermiteSpline(i, j));
        splines2.add(new QuinticHermiteSpline(j, k));
        splines2.add(new QuinticHermiteSpline(k, l));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines2) < 0.05);
        assertEquals(splines2.get(0).getCurvature(1.0), 0.0, kEpsilon);
        assertEquals(splines2.get(2).getCurvature(1.0), 0.0, kEpsilon);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }
}
