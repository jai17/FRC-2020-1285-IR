package frc.team1285.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.team1285.util.NumberConstants;
import frc.team1285.util.DriveMotionPlanner;
import frc.team1285.util.ElectricalConstants;
import frc.team1285.util.kinematics.SwerveHeadingController;
import frc.team1285.util.kinematics.SwerveInverseKinematics;
import frc.team1285.util.kinematics.Util;
import frc.team254.drivers.Pigeon;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

	// Instance declaration
	private static Swerve instance = null;

	public static Swerve getInstance() {
		if (instance == null)
			instance = new Swerve();
		return instance;
	}

	// Module declaration
	public SwerveDriveModule frontRight, frontLeft, rearLeft, rearRight;
	protected List<SwerveDriveModule> modules;
	protected List<SwerveDriveModule> positionModules;

	// Evade maneuver variables
	protected Translation2d clockwiseCenter = new Translation2d();
	protected Translation2d counterClockwiseCenter = new Translation2d();
	protected boolean evading = false;
	protected boolean evadingToggled = false;

	public void toggleEvade() {
		evading = !evading;
		evadingToggled = true;
	}

	// Pigeon
	Pigeon pigeon;

	// Heading controller
	public SwerveHeadingController headingController = new SwerveHeadingController();

	public void temporarilyDisableHeadingController() {
		headingController.temporarilyDisable();
	}

	public double getTargetHeading() {
		return headingController.getTargetHeading();
	}

	// Generator for trajectories
	protected TrajectoryGenerator generator;

	// Odometry variables
	protected Pose2d pose;
	protected double distanceTraveled;
	protected double currentVelocity = 0;
	protected double lastUpdateTimestamp = 0;

	public Pose2d getPose() {
		return pose;
	}

	// Module configuration variables (for beginnning of auto)
	protected boolean modulesReady = false;
	protected boolean alwaysConfigureModules = false;
	protected boolean moduleConfigRequested = false;

	public void requireModuleConfiguration() {
		modulesReady = false;
	}

	public void alwaysConfigureModules() {
		alwaysConfigureModules = true;
	}

	protected Pose2d startingPose = NumberConstants.kRobotLeftStartingPose;

	/**
	 * Set the starting pose of the drive.
	 * 
	 * @param newPose
	 */
	public void setStartingPose(Pose2d newPose) {
		startingPose = newPose;
	}

	// Trajectory variables
	protected DriveMotionPlanner motionPlanner;

	protected double rotationScalar;
	protected double trajectoryStartTime = 0;
	protected Translation2d lastTrajectoryVector = new Translation2d();

	public Translation2d getLastTrajectoryVector() {
		return lastTrajectoryVector;
	}

	protected boolean hasStartedFollowing = false;
	protected boolean hasFinishedPath = false;

	public boolean hasFinishedPath() {
		return hasFinishedPath;
	}

	/**
	 * Constructor.
	 */
	public Swerve() {

		frontRight = new SwerveDriveModule(ElectricalConstants.FRONT_RIGHT_ROTATION,
				ElectricalConstants.FRONT_RIGHT_DRIVE, 0, NumberConstants.kFrontRightEncoderStartingPos,
				NumberConstants.kVehicleToModuleZero, ElectricalConstants.FRONT_RIGHT_ROTATION_ENCODER);
		frontLeft = new SwerveDriveModule(ElectricalConstants.FRONT_LEFT_ROTATION, ElectricalConstants.FRONT_LEFT_DRIVE,
				1, NumberConstants.kFrontLeftEncoderStartingPos, NumberConstants.kVehicleToModuleOne,
				ElectricalConstants.FRONT_LEFT_ROTATION_ENCODER);
		rearLeft = new SwerveDriveModule(ElectricalConstants.REAR_LEFT_ROTATION, ElectricalConstants.REAR_LEFT_DRIVE, 2,
				NumberConstants.kRearLeftEncoderStartingPos, NumberConstants.kVehicleToModuleTwo,
				ElectricalConstants.REAR_LEFT_ROTATION_ENCODER);
		rearRight = new SwerveDriveModule(ElectricalConstants.REAR_RIGHT_ROTATION, ElectricalConstants.REAR_RIGHT_DRIVE,
				3, NumberConstants.kRearRightEncoderStartingPos, NumberConstants.kVehicleToModuleThree,
				ElectricalConstants.REAR_RIGHT_ROTATION_ENCODER);

		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		positionModules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);

		rearLeft.invertDriveMotor(false);
		frontLeft.invertDriveMotor(false);

		modules.forEach((m) -> m.reverseRotationSensor(true));

		pigeon = new Pigeon(ElectricalConstants.PIGEON_IMU);

		pose = new Pose2d();
		distanceTraveled = 0;

		motionPlanner = new DriveMotionPlanner();

		generator = TrajectoryGenerator.getInstance();

		translationalVector = new Translation2d();
		lastDriveVector = rotationalVector;
		rotationalInput = 0;
		headingController.temporarilyDisable();
		lastUpdateTimestamp = Timer.getFPGATimestamp();

	}

	// Assigns appropriate directions for scrub factors
	public void setCarpetDirection(boolean standardDirection) {
		modules.forEach((m) -> m.setCarpetDirection(standardDirection));
	}

	// Teleop driving variables
	protected Translation2d translationalVector = new Translation2d();
	protected double rotationalInput = 0;
	protected Translation2d lastDriveVector = new Translation2d();
	protected final Translation2d rotationalVector = Translation2d.identity();
	protected double lowPowerScalar = 0.6;

	public void setLowPowerScalar(double scalar) {
		lowPowerScalar = scalar;
	}

	protected double maxSpeedFactor = 1.0;

	public void setMaxSpeed(double max) {
		maxSpeedFactor = max;
	}

	protected boolean robotCentric = false;

	// Swerve kinematics (exists in a separate class)
	protected SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();

	public void setCenterOfRotation(Translation2d center) {
		inverseKinematics.setCenterOfRotation(center);
	}

	/** Locks drive motors in place with MotionMagic */
	public void lockDrivePosition() {
		modules.forEach((m) -> m.setDrivePositionTarget(0.0));
	}

	/** Configures each module to match its assigned vector */
	public void setDriveOutput(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
					modules.get(i).getModuleAngle().getDegrees())) {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
			} else {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
			}
		}
	}

	public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride) {
		for (int i = 0; i < modules.size(); i++) {
			if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
					modules.get(i).getModuleAngle().getDegrees())) {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-percentOutputOverride);
			} else {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(percentOutputOverride);
			}
		}
	}

	/**
	 * Configures each module to match its assigned vector, but puts the drive
	 * motors into closed-loop velocity mode
	 */
	public void setVelocityDriveOutput(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
					modules.get(i).getModuleAngle().getDegrees())) {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(
						-driveVectors.get(i).norm() * NumberConstants.kSwerveMaxSpeedInchesPerSecond);
			} else {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(
						driveVectors.get(i).norm() * NumberConstants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}

	public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverride) {
		for (int i = 0; i < modules.size(); i++) {
			if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
					modules.get(i).getModuleAngle().getDegrees())) {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-velocityOverride);
			} else {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(velocityOverride);
			}
		}
	}

	/** Sets only module angles to match their assigned vectors */
	public void setModuleAngles(List<Translation2d> driveVectors) {
		for (int i = 0; i < modules.size(); i++) {
			if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
					modules.get(i).getModuleAngle().getDegrees())) {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
			} else {
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
			}
		}
	}

	/** Increases each module's rotational power cap for the beginning of auto */
	public void set10VoltRotationMode(boolean tenVolts) {
		modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
	}

	/**
	 * @return Whether or not at least one module has reached its MotionMagic
	 *         setpoint
	 */
	public boolean positionOnTarget() {
		boolean onTarget = false;
		for (SwerveDriveModule m : modules) {
			onTarget |= m.drivePositionOnTarget();
		}
		return onTarget;
	}

	/**
	 * @return Whether or not all modules have reached their angle setpoints
	 */
	public boolean moduleAnglesOnTarget() {
		boolean onTarget = true;
		for (SwerveDriveModule m : modules) {
			onTarget &= m.angleOnTarget();
		}
		return onTarget;
	}

	/**
	 * Determines which wheels the robot should rotate about in order to perform an
	 * evasive maneuver
	 */
	public synchronized void determineEvasionWheels() {
		Translation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
		List<Translation2d> wheels = NumberConstants.kModulePositions;
		clockwiseCenter = wheels.get(0);
		counterClockwiseCenter = wheels.get(wheels.size() - 1);
		for (int i = 0; i < wheels.size() - 1; i++) {
			Translation2d cw = wheels.get(i);
			Translation2d ccw = wheels.get(i + 1);
			if (here.isWithinAngle(cw, ccw)) {
				clockwiseCenter = ccw;
				counterClockwiseCenter = cw;
			}
		}
	}

	/** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp) {
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();

		double averageDistance = 0.0;
		double[] distances = new double[4];
		for (SwerveDriveModule m : positionModules) {
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse())
					.norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();

		int minDevianceIndex = 0;
		double minDeviance = 100.0;
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for (SwerveDriveModule m : positionModules) {
			double deviance = Math.abs(distances[m.moduleID] - averageDistance);
			if (deviance < minDeviance) {
				minDeviance = deviance;
				minDevianceIndex = m.moduleID;
			}
			if (deviance <= 0.01) {
				modulesToUse.add(m);
			}
		}

		if (modulesToUse.isEmpty()) {
			modulesToUse.add(modules.get(minDevianceIndex));
		}

		for (SwerveDriveModule m : modulesToUse) {
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	/**
	 * Playing around with different methods of odometry. This will require the use
	 * of all four modules, however.
	 */
	public synchronized void alternatePoseUpdate() {
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();

		double[][] distances = new double[4][2];
		for (SwerveDriveModule m : modules) {
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
			distances[m.moduleID][0] = m.moduleID;
			distances[m.moduleID][1] = distance;
		}

		Arrays.sort(distances, new java.util.Comparator<double[]>() {
			public int compare(double[] a, double[] b) {
				return Double.compare(a[1], b[1]);
			}
		});
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		double firstDifference = distances[1][1] - distances[0][1];
		double secondDifference = distances[2][1] - distances[1][1];
		double thirdDifference = distances[3][1] - distances[2][1];
		if (secondDifference > (1.5 * firstDifference)) {
			modulesToUse.add(modules.get((int) distances[0][0]));
			modulesToUse.add(modules.get((int) distances[1][0]));
		} else if (thirdDifference > (1.5 * firstDifference)) {
			modulesToUse.add(modules.get((int) distances[0][0]));
			modulesToUse.add(modules.get((int) distances[1][0]));
			modulesToUse.add(modules.get((int) distances[2][0]));
		} else {
			modulesToUse.add(modules.get((int) distances[0][0]));
			modulesToUse.add(modules.get((int) distances[1][0]));
			modulesToUse.add(modules.get((int) distances[2][0]));
			modulesToUse.add(modules.get((int) distances[3][0]));
		}

		SmartDashboard.putNumber("Modules Used", modulesToUse.size());

		for (SwerveDriveModule m : modulesToUse) {
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}

		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
		distanceTraveled += deltaPos;
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	public void setNominalDriveOutput(double voltage) {
		modules.forEach((m) -> m.setNominalDriveOutput(voltage));
	}

	/**
	 * Sets the maximum rotation speed of the modules, based on the robot's
	 * velocity
	 */
	public void setMaxRotationSpeed() {
		double currentDriveSpeed = translationalVector.norm() * NumberConstants.kSwerveMaxSpeedInchesPerSecond;
		double newMaxRotationSpeed = NumberConstants.kSwerveRotationMaxSpeed
				/ ((NumberConstants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
		modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
	}

	/**
	 * Zeroes the drive motors, and sets the robot's internal position and heading
	 * to match that of the fed pose
	 */

	public synchronized void zeroSensors() {
		zeroSensors(NumberConstants.kRobotLeftStartingPose);
	}

	public synchronized void zeroSensors(Pose2d startingPose) {
		pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
		modules.forEach((m) -> m.zeroSensors(startingPose));
		pose = startingPose;
		distanceTraveled = 0;
	}

	/** Robot Position Methods **/

	public synchronized void resetPosition(Pose2d newPose) {
		pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
		modules.forEach((m) -> m.zeroSensors(pose));
		distanceTraveled = 0;
	}

	public synchronized void setXCoordinate(double x) {
		pose.getTranslation().setX(x);
		modules.forEach((m) -> m.zeroSensors(pose));
		System.out.println("X coordinate reset to: " + pose.getTranslation().x());
	}

	public synchronized void setYCoordinate(double y) {
		pose.getTranslation().setY(y);
		modules.forEach((m) -> m.zeroSensors(pose));
		System.out.println("Y coordinate reset to: " + pose.getTranslation().y());
	}

	/** Gyro Methods **/

	public double getYaw() {
		return pigeon.getYaw().getDegrees();
	}

	public double getPitch() {
		return pigeon.getPitch();
	}

	public double getRoll() {
		return pigeon.getRoll();
	}

	/** Output drive information to SmartDashboard **/
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		SmartDashboard.putNumberArray("Robot Pose", new double[] { pose.getTranslation().x(), pose.getTranslation().y(),
				pose.getRotation().getUnboundedDegrees() });
		if (true) {
			SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
			SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
			SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees());
			SmartDashboard.putString("Heading Controller", headingController.getState().toString());
			SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
			SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
			SmartDashboard.putNumber("Robot Velocity", currentVelocity);
			SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());
		}
	}
}
