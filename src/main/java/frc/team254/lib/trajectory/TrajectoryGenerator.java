package frc.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.team1285.util.NumberConstants;
import frc.team1285.util.DriveMotionPlanner;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.trajectory.timing.TimedState;
import frc.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0;
    private static final double kMaxDecel = 72.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println(
                    "Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel,
                max_voltage, default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                max_accel, max_decel, max_voltage, default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle
    // of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x
    // axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(
            NumberConstants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)),
            Rotation2d.fromDegrees(-90.0));

    static final Pose2d closeHatchScoringPose = NumberConstants.closeHatchPosition
            .transformBy(Pose2d.fromTranslation(new Translation2d(-NumberConstants.kRobotHalfLength - 3.5, -2.0)));
    static final Pose2d humanLoaderPose = NumberConstants.humanLoaderPosition
            .transformBy(Pose2d.fromTranslation(new Translation2d(NumberConstants.kRobotHalfLength - 4.0, 2.0)));
    static final Pose2d closeShipScoringPose = NumberConstants.closeShipPosition
            .transformBy(Pose2d.fromTranslation(new Translation2d(-NumberConstants.kRobotHalfLength - 4.0, 6.0)));
    static final Pose2d portScoringPose = NumberConstants.rocketPortPosition
            .transformBy(Pose2d.fromTranslation(new Translation2d(-NumberConstants.kRobotHalfLength - 6.0, 0.0)));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        // Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;

        // Auto Paths
        public final MirroredTrajectory startToCloseShip;
        public final MirroredTrajectory closeShipToHumanLoader;

        private TrajectorySet() {

            straightPath = getStraightPath();

            startToCloseShip = new MirroredTrajectory(getStartToCloseShip());
            closeShipToHumanLoader = new MirroredTrajectory(getCloseShipToHumanLoader());
        }

        /********* PATHS (VIA WAYPOINTS) *********/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(NumberConstants.kRobotLeftStartingPose);
            waypoints.add(NumberConstants.kRobotLeftStartingPose
                    .transformBy(Pose2d.fromTranslation(new Translation2d(72.0, 0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCloseShipToHumanLoader() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(closeShipScoringPose);
            waypoints.add(new Pose2d(
                    portScoringPose.transformBy(Pose2d.fromTranslation(new Translation2d(-6.0, 0.0))).getTranslation(),
                    Rotation2d.fromDegrees(0.0)));
            waypoints.add(humanLoaderPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 120.0, kMaxAccel, 24.0, kMaxVoltage, 72.0, 20);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToCloseShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new
            // Pose2d(portScoringPose.transformBy(Pose2d.fromTranslation(new
            // Translation2d(-6.0, 0.0))).getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(closeShipScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), 120.0, kMaxAccel, 24.0, kMaxVoltage, 72.0, 20);
        }
    }

}
