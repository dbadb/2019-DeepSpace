package com.spartronics4915.frc2019.planners;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.Kinematics;
import com.spartronics4915.lib.geometry.Pose2;
import com.spartronics4915.lib.geometry.Rotation2;
import com.spartronics4915.lib.geometry.Translation2;
import com.spartronics4915.lib.geometry.Twist2;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.CentripetalAccelerationConstraint;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class DriveMotionPlannerTest
{

    @Test
    public void testForwardSwerveRight()
    {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2(new Translation2(0.0, 0.0), Rotation2.identity()),
                        new Pose2(new Translation2(120.0, -36.0), Rotation2.identity()),
                        new Pose2(new Translation2(240.0, -36.0), Rotation2.identity())),
                Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                120.0, 120.0, 10.0))));

        double t = 0.0;
        Pose2 pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone())
        {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();//.transformBy(new Pose2(new Translation2(0.0, 1.0),
            // Rotation2.fromDegrees(2.0)));

            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;// + (2.0 * Math.random() - 1.0) * 0.002;
        }
    }

    @Test
    public void testForwardSwerveLeft()
    {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2(new Translation2(0.0, 0.0), Rotation2.identity()),
                        new Pose2(new Translation2(120.0, 36.0), Rotation2.identity()),
                        new Pose2(new Translation2(240.0, 0.0), Rotation2.identity())),
                null,
                120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2 pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone())
        {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();//.transformBy(new Pose2(new Translation2(0.0, -1.0),
            // Rotation2.fromDegrees(-2.0)));
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testReverseSwerveLeft()
    {
        DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(true,
                Arrays.asList(new Pose2(new Translation2(240.0, 0.0), Rotation2.identity()),
                        new Pose2(new Translation2(120.0, 36.0), Rotation2.identity()),
                        new Pose2(new Translation2(0.0, 0.0), Rotation2.identity())),
                null,
                120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2 pose = motion_planner.setpoint().state().getPose();
        while (!motion_planner.isDone())
        {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }

    @Test
    public void testForwardReverseSame()
    {
        DriveMotionPlanner fwd_motion_planner = new DriveMotionPlanner();
        fwd_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        fwd_motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(fwd_motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2(new Translation2(10.0, 10.0), Rotation2.fromDegrees(5.0)),
                        new Pose2(new Translation2(120.0, 36.0), Rotation2.identity()),
                        new Pose2(new Translation2(230.0, 10.0), Rotation2.fromDegrees(-5.0))),
                null,
                120.0, 120.0, 5.0))));
        DriveMotionPlanner rev_motion_planner = new DriveMotionPlanner();
        rev_motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        rev_motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(rev_motion_planner.generateTrajectory(true,
                Arrays.asList(new Pose2(new Translation2(230.0, 10.0), Rotation2.fromDegrees(-5.0)),
                        new Pose2(new Translation2(120.0, 36.0), Rotation2.identity()),
                        new Pose2(new Translation2(10.0, 10.0), Rotation2.fromDegrees(5.0))),
                null,
                120.0, 120.0, 5.0))));

        final double dt = 0.01;
        double t = 0.0;
        Pose2 start_error_fwd = new Pose2(2.0, 3.0, Rotation2.fromDegrees(2.0));
        Pose2 start_error_rev = new Pose2(-2.0, 3.0, Rotation2.fromDegrees(-2.0));
        Pose2 fwd_pose = fwd_motion_planner.setpoint().state().getPose().transformBy(start_error_fwd);
        Pose2 rev_pose = rev_motion_planner.setpoint().state().getPose().transformBy(start_error_rev);
        while (!fwd_motion_planner.isDone() || !rev_motion_planner.isDone())
        {
            DriveMotionPlanner.Output fwd_output = fwd_motion_planner.update(t, fwd_pose);
            Twist2 fwd_delta = Kinematics.forwardKinematics(fwd_output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    fwd_output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            fwd_pose = fwd_pose.transformBy(Pose2.exp(fwd_delta));
            //System.out.println("FWD Delta: " + fwd_delta + ", Pose: " + fwd_pose);
            DriveMotionPlanner.Output rev_output = rev_motion_planner.update(t, rev_pose);
            Twist2 rev_delta = Kinematics.forwardKinematics(rev_output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    rev_output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            rev_pose = rev_pose.transformBy(Pose2.exp(rev_delta));
            //System.out.println("REV Delta: " + rev_delta + ", Pose: " + rev_pose);
            System.out.println(fwd_motion_planner.toCSV() + "," + rev_motion_planner.toCSV());
            t += dt;
        }
    }

    @Test
    public void testFollowerReachesGoal()
    {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(new Pose2(new Translation2(0.0, 0.0), Rotation2.identity()),
                        new Pose2(new Translation2(120.0, -36.0), Rotation2.identity()),
                        new Pose2(new Translation2(240.0, -36.0), Rotation2.identity())),
                null,
                120.0, 120.0, 10.0))));
        final double dt = 0.01;
        double t = 0.0;
        Pose2 initial_error = new Pose2(2.0, 3.0, Rotation2.fromDegrees(3.5));
        Pose2 pose = motion_planner.setpoint().state().getPose().transformBy(initial_error);
        while (!motion_planner.isDone())
        {
            DriveMotionPlanner.Output output = motion_planner.update(t, pose);
            Twist2 delta = Kinematics.forwardKinematics(output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            // Add some systemic error.
            delta = new Twist2(delta.dx * 1.0, delta.dy * 1.0, delta.dtheta * 1.05);
            pose = pose.transformBy(Pose2.exp(delta));
            t += dt;
            System.out.println(motion_planner.setpoint().toCSV() + "," + pose.toCSV());
        }
        System.out.println(pose);
    }

    @Test
    public void testVoltages()
    {
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false,
                Arrays.asList(Pose2.identity(), Pose2.fromTranslation(new Translation2(48.0, 0.0)),
                        new Pose2(new Translation2(96.0, 48.0), Rotation2.fromDegrees(90.0)),
                        new Pose2(new Translation2(96.0, 96.0), Rotation2.fromDegrees(90.0))),
                null,
                48.0, 48.0, 10.0))));
        double t = 0.0;
        Pose2 pose = motion_planner.setpoint().state().getPose().transformBy(new Pose2(Translation2.identity(),
                Rotation2.fromDegrees(180.0)));
        while (!motion_planner.isDone())
        {
            motion_planner.update(t, pose);
            pose = motion_planner.mSetpoint.state().getPose();
            System.out.println(t + "," + motion_planner.toCSV());
            t += 0.01;
        }
    }
}
