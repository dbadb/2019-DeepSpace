package com.spartronics4915.frc2019.paths;

import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Pose2WithCurvature;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.TrajectoryUtil;
import com.spartronics4915.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.trajectory.timing.TimingConstraint;
import com.spartronics4915.lib.util.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator
{

    private static final double kMaxVelocity = 24.0; // inches/s
    private static final double kMaxAccel = 24.0; // inches/s
    private static final double kMaxCentripetalAccel = 100.0; // inches/s
    private static final double kMaxVoltage = 9.0; // volts

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance()
    {
        return mInstance;
    }

    private TrajectoryGenerator()
    {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories()
    {
        if (mTrajectorySet == null)
        {
            Logger.debug("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            Logger.debug("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet()
    {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2WithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2> waypoints,
            final List<TimingConstraint<Pose2WithCurvature>> constraints,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2WithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2> waypoints,
            final List<TimingConstraint<Pose2WithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    // TODO: Add critical poses here

    public class TrajectorySet
    {

        public class MirrorableTrajectory
        {
            public MirrorableTrajectory(Trajectory<TimedState<Pose2WithCurvature>> right)
            {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2WithCurvature>> get(boolean left)
            {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2WithCurvature>> left;
            public final Trajectory<TimedState<Pose2WithCurvature>> right;
        }

        public final MirrorableTrajectory straightTest;
        public final MirrorableTrajectory curvedTest;

        private TrajectorySet()
        {
            straightTest = new MirrorableTrajectory(getStraightTest());
            curvedTest = new MirrorableTrajectory(getCurvedTest());
        }

        private Trajectory<TimedState<Pose2WithCurvature>> getStraightTest()
        {
            List<Pose2> waypoints = new ArrayList<>();
            waypoints.add(new Pose2(0d, 0d, Rotation2.identity()));
            waypoints.add(new Pose2(60d, 0d, Rotation2.identity()));
            return generateTrajectory(false, waypoints, 
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2WithCurvature>> getCurvedTest()
        {
            List<Pose2> waypoints = new ArrayList<>();
            waypoints.add(new Pose2(0d, 0d, Rotation2.identity()));
            waypoints.add(new Pose2(78d, 78d, Rotation2.fromDegrees(90)));
            return generateTrajectory(false, waypoints, 
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

    }
}
