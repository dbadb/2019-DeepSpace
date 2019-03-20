package com.spartronics4915.frc2019;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;

public class TestSystemTest
{

    @Test
    public void aTest()
    {
        // assert statements
        assertEquals(0, 0, "0 must be 0");

        System.out.println(Constants.ScorableLandmark.LEFT_CLOSE_CARGO_BAY.fieldPose);
        System.out.println(Constants.kRightRobotLocationOnPlatform);
    }
}
