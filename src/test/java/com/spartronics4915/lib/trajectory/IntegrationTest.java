package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Pose2WithCurvature;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.physics.DCMotorTransmission;
import com.spartronics4915.lib.physics.DifferentialDrive;
import com.spartronics4915.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.trajectory.timing.TimingUtil;
import com.spartronics4915.lib.util.Units;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

public class IntegrationTest
{

        @Test
        public void testFollowerTrajectoryGenerator()
        {
                // Specify desired waypoints.
                List<Translation2> waypoints = Arrays.asList(
                                new Translation2(0.0, 0.0),
                                new Translation2(24.0, 0.0),
                                new Translation2(36.0, 0.0),
                                new Translation2(36.0, 24.0),
                                new Translation2(60.0, 24.0));

                // Create the reference trajectory (straight line motion between waypoints).
                Trajectory<Translation2> reference_trajectory = new Trajectory<>(waypoints);

                // Generate a smooth (continuous curvature) path to follow.
                IPathFollower path_follower = new PurePursuitController<Translation2>(
                                new DistanceView<>(reference_trajectory), /* sampling_dist */1.0, /* lookahead= */ 6.0,
                                /* goal_tolerance= */ 0.1);
                Trajectory<Pose2WithCurvature> smooth_path = TrajectoryUtil.trajectoryFromPathFollower(path_follower,
                                Pose2WithCurvature.identity(), /* step_size= */ 1.0, /* dcurvature_limit= */1.0);

                assertFalse(smooth_path.isEmpty());
                System.out.println(smooth_path.toCSV());

                // Time parameterize the path subject to our dynamic constraints.
                // TODO
        }

        @Test
        public void testSplineTrajectoryGenerator()
        {
                // Specify desired waypoints.
                List<Pose2> waypoints = Arrays.asList(
                                new Pose2(0.0, 0.0, Rotation2.fromDegrees(0.0)),
                                new Pose2(36.0, 0.0, Rotation2.fromDegrees(0.0)),
                                new Pose2(60.0, 100, Rotation2.fromDegrees(0.0)),
                                new Pose2(160.0, 100.0, Rotation2.fromDegrees(0.0)),
                                new Pose2(200.0, 70, Rotation2.fromDegrees(45.0)));

                // Create a trajectory from splines.
                Trajectory<Pose2WithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints, 2.0,
                                0.2, Math.toRadians(5.0));
                // System.out.println(trajectory.toCSV());
                // Create a differential drive.
                final double kRobotMassKg = 60.0;
                final double kRobotAngularInertia = 80.0;
                final double kRobotAngularDrag = 0.0;
                final double kWheelRadius = Units.inches_to_meters(2.0);
                DCMotorTransmission transmission = new DCMotorTransmission(
                                1.0 / 0.143,
                                (kWheelRadius * kWheelRadius * kRobotMassKg / 2.0) / 0.02,
                                0.8);
                DifferentialDrive drive = new DifferentialDrive(kRobotMassKg, kRobotAngularInertia, kRobotAngularDrag, kWheelRadius, Units
                                .inches_to_meters(26.0 / 2.0), transmission, transmission);

                // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
                // than 10V.
                DifferentialDriveDynamicsConstraint<Pose2WithCurvature> drive_constraints = new DifferentialDriveDynamicsConstraint<>(drive, 10.0);

                // Generate the timed trajectory.
                Trajectory<TimedState<Pose2WithCurvature>> timed_trajectory =
                                TimingUtil.timeParameterizeTrajectory(false, new DistanceView<>(trajectory), 2.0, Arrays.asList(drive_constraints),
                                                0.0, 0.0, 12.0 * 14.0, 12.0 * 10.0);

                // System.out.println(timed_trajectory.toCSV());
                for (int i = 1; i < timed_trajectory.length(); ++i)
                {
                        TrajectoryPoint<TimedState<Pose2WithCurvature>> prev = timed_trajectory.getPoint(i - 1);
                        TrajectoryPoint<TimedState<Pose2WithCurvature>> next = timed_trajectory.getPoint(i);
                        assertEquals(prev.state().acceleration(), (next.state().velocity() - prev.state().velocity()) / (next
                                        .state().t() - prev.state().t()), 1E-9);
                        final double dt = next.state().t() - prev.state().t();
                        assertEquals(next.state().velocity(), prev.state().velocity() + prev.state().acceleration() * dt, 1E-9);
                        assertEquals(next.state().distance(prev.state()), prev.state().velocity() * dt + 0.5 * prev.state()
                                        .acceleration() * dt * dt, 1E-9);
                }

                // "Follow" the trajectory.
                final double kDt = 0.01;
                boolean first = true;
                TrajectoryIterator<TimedState<Pose2WithCurvature>> it = new TrajectoryIterator<>(new TimedView<>(timed_trajectory));
                while (!it.isDone())
                {
                        TrajectorySamplePoint<TimedState<Pose2WithCurvature>> sample;
                        if (first)
                        {
                                sample = it.getSample();
                                first = false;
                        }
                        else
                        {
                                sample = it.advance(kDt);
                        }
                        final TimedState<Pose2WithCurvature> state = sample.state();

                        final DifferentialDrive.DriveDynamics dynamics = drive.solveInverseDynamics(
                                        new DifferentialDrive.ChassisState(Units.inches_to_meters(state.velocity()), state.velocity() *
                                                        state.state().getCurvature()),
                                        new DifferentialDrive.ChassisState(Units.inches_to_meters(state.acceleration()), state
                                                        .acceleration() * state.state().getCurvature()));

                        System.out.println(state.toCSV() + ", " + dynamics.toCSV());
                }
        }

}
