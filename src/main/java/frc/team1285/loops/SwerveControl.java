package frc.team1285.loops;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.team1285.robot.RobotState;
import frc.team1285.subsystems.Swerve;
import frc.team1285.util.NumberConstants;
import frc.team1285.util.kinematics.InterpolatingDouble;
import frc.team1285.util.kinematics.Util;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.trajectory.TimedView;
import frc.team254.lib.trajectory.Trajectory;
import frc.team254.lib.trajectory.TrajectoryIterator;
import frc.team254.lib.trajectory.timing.TimedState;

public class SwerveControl extends Swerve {

    private static SwerveControl mInstance;
    RobotState robotState;

    private NetworkTableEntry controlMode;

    // Create a single instance of the Swerve Control
    public static SwerveControl getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveControl();
        }
        return mInstance;
    }

    /**
     * Constructor. Creates an instance of the parent Subsystem Swerve
     */
    private SwerveControl() {
        super();
        controlMode = Shuffleboard.getTab("Control Loops").add("Swerve Control", 0).getEntry();
        robotState = RobotState.getInstance();
    }

    /** Swerve control states */
    public enum SwerveControlState {
        NEUTRAL("Neutral"), MANUAL("Manual"), POSITION("Position"), ROTATION("Rotation"), DISABLED("Disabled"),
        TRAJECTORY("Trajectory"), VELOCITY("Velocity");

        public final String label;

        private SwerveControlState(String label) {
            this.label = label;
        }
    }

    private SwerveControlState mControlState = SwerveControlState.NEUTRAL;

    @Override
    public void periodic() {
        controlMode.setString(mControlState.label);
        double rotationCorrection = this.headingController
                .updateRotationCorrection(pose.getRotation().getUnboundedDegrees(), Timer.getFPGATimestamp());
        if (modulesReady || (getControlState() != SwerveControlState.TRAJECTORY)) {
            // updatePose(timestamp);
            alternatePoseUpdate();
        }
        // Set the max speed of the drive, can be dependant on other limits (Ex. ElevatorHeight)
        setMaxSpeed(NumberConstants.kSwerveSpeedTreeMap.getInterpolated(new InterpolatingDouble(0.0)).value);
        robotState.addFieldToVehicleObservation(Timer.getFPGATimestamp(), getPose());

        switch (mControlState) {
            case MANUAL:
                configureEvasion();
                if (translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0) {
                    if (lastDriveVector.equals(rotationalVector)) {
                        stop();
                    } else {
                        setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector, rotationCorrection, pose,
                                robotCentric), 0.0);
                    }
                } else {
                    setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
                            rotationalInput + rotationCorrection, pose, robotCentric));
                }
                break;
            case POSITION:
                if (positionOnTarget())
                    rotate(headingController.getTargetHeading());
                break;
            case ROTATION:
                setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(),
                        Util.deadBand(rotationCorrection, 0.1), pose, false));
                break;
            case TRAJECTORY:
                if (!motionPlanner.isDone()) {
                    Translation2d driveVector = motionPlanner.update(this.lastUpdateTimestamp, pose);

                    if (modulesReady) {
                        if (!hasStartedFollowing) {
                            if (moduleConfigRequested) {
                                zeroSensors(startingPose);
                                System.out.println("Position reset for auto");
                            }
                            hasStartedFollowing = true;
                        }
                        double rotationInput = Util
                                .deadBand(Util.limit(rotationCorrection * rotationScalar * driveVector.norm(),
                                        motionPlanner.getMaxRotationSpeed()), 0.01);
                        if (Util.epsilonEquals(driveVector.norm(), 0.0, NumberConstants.kEpsilon)) {
                            driveVector = lastTrajectoryVector;
                            setVelocityDriveOutput(
                                    inverseKinematics.updateDriveVectors(driveVector, rotationInput, pose, false), 0.0);
                            // System.out.println("Trajectory Vector set: " + driveVector.toString());
                        } else {
                            setVelocityDriveOutput(
                                    inverseKinematics.updateDriveVectors(driveVector, rotationInput, pose, false));
                            // System.out.println("Trajectory Vector set: " + driveVector.toString());
                        }
                    } else if (!moduleConfigRequested) {
                        // set10VoltRotationMode(true);
                        setModuleAngles(inverseKinematics.updateDriveVectors(driveVector, 0.0, pose, false));
                        moduleConfigRequested = true;
                    }

                    if (moduleAnglesOnTarget() && !modulesReady) {
                        set10VoltRotationMode(false);
                        modules.forEach((m) -> m.resetLastEncoderReading());
                        modulesReady = true;
                        System.out.println("Modules Ready");
                    }

                    lastTrajectoryVector = driveVector;
                } else {
                    if (!hasFinishedPath) {
                        System.out.println("Path completed in: " + (Timer.getFPGATimestamp() - trajectoryStartTime));
                        hasFinishedPath = true;
                        if (alwaysConfigureModules)
                            requireModuleConfiguration();
                    }
                }
                break;
            case VELOCITY:
                break;
            case NEUTRAL:
                stop();
                break;
            case DISABLED:
                break;
            default:
                break;
        }
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

    // Set the swerve control state
    public void setControlState(SwerveControlState mControlState) {
        this.mControlState = mControlState;
    }

    // Get the current swerve state
    public SwerveControlState getControlState() {
        return mControlState;
    }

    /**
     * @return remaiing progress of the motion planner
     */
    public double getRemainingProgress() {
        if (motionPlanner != null && getControlState() == SwerveControlState.TRAJECTORY) {
            return motionPlanner.getRemainingProgress();
        }
        return 0.0;
    }

    /**
     * Main function used to send manual input during teleop.
     * 
     * @param x            forward/backward input
     * @param y            left/right input
     * @param rotate       rotational input
     * @param robotCentric gyro use
     * @param lowPower     scaled down output
     */
    public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower) {

        Translation2d translationalInput = new Translation2d(x, y);
        double inputMagnitude = translationalInput.norm();

        /*
         * Snap the translational input to its nearest pole, if it is within a certain
         * threshold of it.
         */
        double threshold = Math.toRadians(10.0);
        if (Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold) {
            translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
        }

        double deadband = 0.15;
        if (inputMagnitude < deadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        /*
         * Scale x and y by applying a power to the magnitude of the vector they create,
         * in order to make the controls less sensitive at the lower end.
         */
        final double power = (lowPower) ? 1.75 : 1.5;
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude, direction.sin() * scaledMagnitude);

        rotate = (Math.abs(rotate) < deadband) ? 0 : rotate;
        rotate = Math.pow(Math.abs(rotate), 1.75) * Math.signum(rotate);

        translationalInput = translationalInput.scale(maxSpeedFactor);
        rotate *= maxSpeedFactor;

        translationalVector = translationalInput;

        if (lowPower) {
            translationalVector = translationalVector.scale(lowPowerScalar);
            rotate *= lowPowerScalar;
        } else {
            rotate *= 0.8;
        }

        if (rotate != 0 && rotationalInput == 0) {
            headingController.disable();
        } else if (rotate == 0 && rotationalInput != 0) {
            headingController.temporarilyDisable();
        }

        rotationalInput = rotate;

        setControlState(SwerveControlState.MANUAL);

        if (inputMagnitude > 0.3)
            lastDriveVector = new Translation2d(x, y);
        else if (translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0) {
            lastDriveVector = rotationalVector;
        }

        this.robotCentric = robotCentric;
    }

    /** Setup for an evasion maneuver */
    public void configureEvasion(){
        if (evading && evadingToggled) {
            determineEvasionWheels();
            double sign = Math.signum(rotationalInput);
            if (sign == 1.0) {
                inverseKinematics.setCenterOfRotation(clockwiseCenter);
            } else if (sign == -1.0) {
                inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
            }
            evadingToggled = false;
        } else if (evading) {
            double sign = Math.signum(rotationalInput);
            if (sign == 1.0) {
                inverseKinematics.setCenterOfRotation(clockwiseCenter);
            } else if (sign == -1.0) {
                inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
            }
        } else if (evadingToggled) {
            inverseKinematics.setCenterOfRotation(Translation2d.identity());
            evadingToggled = false;
        }
    }

    // Various methods to control the heading controller
    public synchronized void rotate(double goalHeading) {
        if (translationalVector.x() == 0 && translationalVector.y() == 0)
            rotateInPlace(goalHeading);
        else
            headingController.setStabilizationTarget(
                    Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    public void setPathHeading(double goalHeading) {
        headingController.setSnapTarget(
                Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    public void setAbsolutePathHeading(double absoluteHeading) {
        headingController.setSnapTarget(absoluteHeading);
    }

    /** Sets MotionMagic targets for the drive motors */
    public void setPositionTarget(double directionDegrees, double magnitudeInches) {
        setControlState(SwerveControlState.POSITION);
        modules.forEach((m) -> m.setModuleAngle(directionDegrees));
        modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
    }

    public void rotateInPlace(double goalHeading) {
        setControlState(SwerveControlState.ROTATION);
        headingController.setStationaryTarget(
                Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
    }

    public void rotateInPlaceAbsolutely(double absoluteHeading) {
        setControlState(SwerveControlState.ROTATION);
        headingController.setStationaryTarget(absoluteHeading);
    }

    /** Puts drive motors into closed-loop velocity mode */
    public void setVelocity(Rotation2d direction, double velocityInchesPerSecond) {
        setControlState(SwerveControlState.VELOCITY);
        modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
        modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
    }

    /**
     * Sets a trajectory for the robot to follow
     * 
     * @param trajectory
     * @param targetHeading   Heading that the robot will rotate to during its path
     *                        following
     * @param rotationScalar  Scalar to increase or decrease the robot's rotation
     *                        speed
     * @param followingCenter The point (relative to the robot) that will follow the
     *                        trajectory
     */
    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
            double rotationScalar, Translation2d followingCenter) {
        hasStartedFollowing = false;
        hasFinishedPath = false;
        moduleConfigRequested = false;
        motionPlanner.reset();
        motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
        motionPlanner.setFollowingCenter(followingCenter);
        inverseKinematics.setCenterOfRotation(followingCenter);
        setAbsolutePathHeading(targetHeading);
        this.rotationScalar = rotationScalar;
        trajectoryStartTime = Timer.getFPGATimestamp();
        setControlState(SwerveControlState.TRAJECTORY);
    }

    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
            double rotationScalar) {
        setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
    }

    public synchronized void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading) {
        setRobotCentricTrajectory(relativeEndPos, targetHeading, 45.0);
    }

    public synchronized void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading,
            double defaultVel) {
        modulesReady = true;
        Translation2d endPos = pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation();
        Rotation2d startHeading = endPos.translateBy(pose.getTranslation().inverse()).direction();
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(pose.getTranslation(), startHeading));
        waypoints.add(
                new Pose2d(pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation(), startHeading));
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints,
                Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
        double heading = Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), targetHeading);
        setTrajectory(trajectory, heading, 1.0);
    }

    /** Puts all rotation and drive motors into open-loop mode */
    public void disable() {
        modules.forEach((m) -> m.disable());
        setControlState(SwerveControlState.DISABLED);
    }

    public void stop() {
        translationalVector = new Translation2d();
        rotationalInput = 0;
        setControlState(SwerveControlState.NEUTRAL);
        modules.forEach((m) -> m.stop());
    }

}