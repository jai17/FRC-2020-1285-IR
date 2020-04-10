package frc.team1285.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.sensors.CANCoder;

import frc.team1285.util.NumberConstants;
import frc.team1285.util.kinematics.Util;
import frc.team254.drivers.LazyTalonFX;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem class representing one swerve drive module, handling encoder values,
 * inputs, outputs and pose calculation.
 */

public class SwerveDriveModule extends SubsystemBase {

	LazyTalonFX rotationMotor, driveMotor;
	CANCoder rotationEncoder;
	int moduleID;
	String name = "Module ";
	int rotationSetpoint = 0;
	double driveSetpoint = 0;
	int encoderOffset;
	int encoderReverseFactor = 1;
	boolean tenVoltRotationMode = false;
	private double previousEncDistance = 0;
	private Translation2d position;
	private Translation2d startingPosition;
	private Pose2d estimatedRobotPose = new Pose2d();
	boolean standardCarpetDirection = true;

	/**
	 * Set the carper direction to account for wheel scrub in pose calculations
	 * @param standardDirection (True if standard direction)
	 */
	public void setCarpetDirection(boolean standardDirection) {
		standardCarpetDirection = standardDirection;
	}

	// Instance of the base inputs and outputs for the module
	PeriodicIO periodicIO = new PeriodicIO();

	/**
	 * Constructor.
	 * 
	 * Creates a single module subsystem
	 * @param rotationSlot The canID of the rotation motor 
	 * @param driveSlot The canID of the drive motor
	 * @param moduleID The assigned ID of the module
	 * @param encoderOffset The starting encoder offset (wheel at 0 degs) if using an absolute encoder
	 * @param startingPose The starting physical pose of the module on the robot
	 * @param rotationEncoderID The canID of the rotation encoder (assuming CTRE CANCoder)
	 */
	public SwerveDriveModule(int rotationSlot, int driveSlot, int moduleID, int encoderOffset,
			Translation2d startingPose, int rotationEncoderID) {
		name += (moduleID + " ");
		rotationMotor = new LazyTalonFX(rotationSlot);
		driveMotor = new LazyTalonFX(driveSlot);
		rotationEncoder = new CANCoder(rotationEncoderID);
		configureMotors();
		this.moduleID = moduleID;
		this.encoderOffset = encoderOffset;
		previousEncDistance = 0;
		position = startingPose;
		this.startingPosition = startingPose;
		getRawAngle();
	}

	/**
	 * Inverts the drive motor input.
	 * 
	 * @param invert
	 */
	public synchronized void invertDriveMotor(boolean invert) {
		driveMotor.setInverted(invert);
	}

	/**
	 * Inverts the rotation motor input.
	 * 
	 * @param invert
	 */
	public synchronized void invertRotationMotor(boolean invert) {
		rotationMotor.setInverted(invert);
	}

	/**
	 * Inverts the drive sensor phase.
	 * 
	 * @param reverse
	 */
	public synchronized void reverseDriveSensor(boolean reverse) {
		driveMotor.setSensorPhase(reverse);
	}

	/**
	 * Inverts the the rotation sensor phase.
	 * 
	 * @param reverse
	 */
	public synchronized void reverseRotationSensor(boolean reverse) {
		encoderReverseFactor = reverse ? -1 : 1;
		rotationMotor.setSensorPhase(reverse);
	}

	/**
	 * Sets the minimum drive output based on desired voltage.
	 * 
	 * @param voltage
	 */
	public synchronized void setNominalDriveOutput(double voltage) {
		driveMotor.configNominalOutputForward(voltage / 12.0, 10);
		driveMotor.configNominalOutputReverse(-voltage / 12.0, 10);
	}

	/**
	 * Set the max rotation speed of the module for Motion Magic.
	 * 
	 * @param maxSpeed in native encoder units
	 */
	public synchronized void setMaxRotationSpeed(double maxSpeed) {
		rotationMotor.configMotionCruiseVelocity((int) maxSpeed, 0);
	}

	/**
	 * Configuration for the drive and rotation motors, encoders and PID.
	 * 
	 * TODO: 
	 * Check that setting up the CANCoder as the remote sensor works
	 */
	private void configureMotors() {
		rotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0);
		rotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);
		rotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 1, 10);
		rotationMotor.setSensorPhase(true);
		rotationMotor.setInverted(false);
		rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
		rotationMotor.setNeutralMode(NeutralMode.Brake);
		rotationMotor.configVoltageCompSaturation(7.0, 10);
		rotationMotor.enableVoltageCompensation(true);
		rotationMotor.configAllowableClosedloopError(0, 0, 10);
		rotationMotor.configMotionAcceleration((int) (NumberConstants.kSwerveRotationMaxSpeed * 12.5), 10);
		rotationMotor.configMotionCruiseVelocity((int) (NumberConstants.kSwerveRotationMaxSpeed), 10);
		rotationMotor.selectProfileSlot(0, 0);
		// Slot 1 is for normal use
		rotationMotor.config_kP(0, NumberConstants.SWERVE_ROTATION_P, 10);
		rotationMotor.config_kI(0, NumberConstants.SWERVE_ROTATION_I, 10);
		rotationMotor.config_kD(0, NumberConstants.SWERVE_ROTATION_D, 10);
		rotationMotor.config_kF(0, 1023.0 / NumberConstants.kSwerveRotationMaxSpeed, 10);
		// Slot 2 is reserved for the beginning of auto (Full Battery)
		rotationMotor.config_kP(1, NumberConstants.SWERVE_ROTATION_P, 10);
		rotationMotor.config_kI(1, NumberConstants.SWERVE_ROTATION_I, 10);
		rotationMotor.config_kD(1, NumberConstants.SWERVE_ROTATION_D, 10);
		rotationMotor.config_kF(1, 1023.0 / NumberConstants.kSwerveRotation10VoltMaxSpeed, 10);
		rotationMotor.set(TalonFXControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));

		driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
		driveMotor.setSelectedSensorPosition(0, 0, 10);
		driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
		driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
		driveMotor.configVelocityMeasurementWindow(32, 10);
		driveMotor.configNominalOutputForward(1.5 / 12.0, 10);
		driveMotor.configNominalOutputReverse(-1.5 / 12.0, 10);
		driveMotor.configVoltageCompSaturation(12.0, 10);
		driveMotor.enableVoltageCompensation(true);
		driveMotor.configOpenloopRamp(0.25, 10);
		driveMotor.configClosedloopRamp(0.0);
		driveMotor.configAllowableClosedloopError(0, 0, 10);
		driveMotor.setInverted(true);
		driveMotor.setSensorPhase(true);
		driveMotor.setNeutralMode(NeutralMode.Brake);
		// Slot 0 is reserved for MotionMagic
		driveMotor.selectProfileSlot(0, 0);
		driveMotor.config_kP(0, NumberConstants.SWERVE_P, 10);
		driveMotor.config_kI(0, NumberConstants.SWERVE_I, 10);
		driveMotor.config_kD(0, NumberConstants.SWERVE_D, 10);
		driveMotor.config_kF(0, 1023.0 / NumberConstants.kSwerveDriveMaxSpeed, 10);
		driveMotor.configMotionCruiseVelocity((int) (NumberConstants.kSwerveDriveMaxSpeed * 0.9), 10);
		driveMotor.configMotionAcceleration((int) (NumberConstants.kSwerveDriveMaxSpeed), 10);
		// Slot 1 corresponds to velocity mode
		driveMotor.config_kP(1, NumberConstants.SWERVE_VEL_P, 10);
		driveMotor.config_kI(1, NumberConstants.SWERVE_VEL_I, 10);
		driveMotor.config_kD(1, NumberConstants.SWERVE_VEL_D, 10);
		driveMotor.config_kF(1, 1023.0 / NumberConstants.kSwerveDriveMaxSpeed * 0.9, 10);
	}

	/**
	 * Get the angle of the module before accounting for initial offset (if set).
	 * 
	 * @return rawModuleAngle
	 */
	private double getRawAngle() {
		return encUnitsToDegrees(periodicIO.rotationPosition);
	}

	/**
	 * Get the angle of the module after accounting for any initial offset (if set).
	 * 
	 * @return moduleAngle 
	 */
	public Rotation2d getModuleAngle() {
		return Rotation2d.fromDegrees(getRawAngle() - encUnitsToDegrees(encoderOffset));
	}

	/**
	 * Get the field centric angle of the module, meaning to take into 
	 * account the current robot heading. 
	 * Ex. If you turn 180 degrees and push forwards the robot will move away from you
	 * 
	 * @param robotHeading
	 * @return field centric angle
	 */
	public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
		Rotation2d normalizedAngle = getModuleAngle();
		return normalizedAngle.rotateBy(robotHeading);
	}

	/**
	 * Sets the desired angle of the module for motion magic control.
	 * 
	 * @param goalAngle
	 */
	public void setModuleAngle(double goalAngle) {
		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(),
				goalAngle + encUnitsToDegrees(encoderOffset));
		int setpoint = degreesToEncUnits(newAngle);
		periodicIO.rotationControlMode = TalonFXControlMode.MotionMagic;
		periodicIO.rotationDemand = setpoint;
	}

	/**
	 * Checks whether the module is at its desired angle demand.
	 * 
	 * @return true if on target
	 */
	public boolean angleOnTarget() {
		double error = encUnitsToDegrees(Math.abs(periodicIO.rotationDemand - periodicIO.rotationPosition));
		return error < 4.0;
	}

	/**
	 * Sets the rotation mode for either tenVolts (Auto) or sevenVolts (Teleop).
	 * 
	 * @param tenVolts true if tenVolts
	 */
	public void set10VoltRotationMode(boolean tenVolts) {
		if (tenVolts && !tenVoltRotationMode) {
			rotationMotor.selectProfileSlot(1, 0);
			rotationMotor.configVoltageCompSaturation(10.0, 10);
			tenVoltRotationMode = true;
		} else if (!tenVolts && tenVoltRotationMode) {
			rotationMotor.selectProfileSlot(0, 0);
			rotationMotor.configVoltageCompSaturation(7.0, 10);
			tenVoltRotationMode = false;
		}
	}

	/**
	 * Sets the rotation control to open loop.
	 * 
	 * @param power
	 */
	public void setRotationOpenLoop(double power) {
		periodicIO.rotationControlMode = TalonFXControlMode.PercentOutput;
		periodicIO.rotationDemand = power;
	}

	/**
	 * Sets the drive control to open loop.
	 * 
	 * @param velocity Normalized value
	 */
	public void setDriveOpenLoop(double velocity) {
		periodicIO.driveControlMode = TalonFXControlMode.PercentOutput;
		periodicIO.driveDemand = velocity;
	}

	/**
	 * Sets the drive position target for motion magic control.
	 * 
	 * @param deltaDistanceInches
	 */
	public void setDrivePositionTarget(double deltaDistanceInches) {
		driveMotor.selectProfileSlot(0, 0);
		periodicIO.driveControlMode = TalonFXControlMode.MotionMagic;
		periodicIO.driveDemand = periodicIO.drivePosition + inchesToEncUnits(deltaDistanceInches);
	}

	/**
	 * Checks whether the drive is at the desired position demand.
	 * 
	 * @return true if on target
	 */
	public boolean drivePositionOnTarget() {
		if (periodicIO.driveControlMode == TalonFXControlMode.MotionMagic)
			return encUnitsToInches((int) Math.abs(periodicIO.driveDemand - periodicIO.drivePosition)) < 2.0;
		return false;
	}

	/**
	 * Sets the desired velocity for velocity control
	 * @param inchesPerSecond
	 */
	public void setVelocitySetpoint(double inchesPerSecond) {
		driveMotor.selectProfileSlot(1, 0);
		periodicIO.driveControlMode = TalonFXControlMode.Velocity;
		periodicIO.driveDemand = inchesPerSecondToEncVelocity(inchesPerSecond);
	}

	/**
	 * @return distance driven in inches
	 */
	private double getDriveDistanceInches() {
		return encUnitsToInches(periodicIO.drivePosition);
	}

	/**
	 * Converts raw encoder units to inches based on conversion factor.
	 * 
	 * @param encUnits
	 * @return
	 */
	public double encUnitsToInches(double encUnits) {
		return encUnits / NumberConstants.kSwerveEncUnitsPerInch;
	}

	/**
	 * Converts inches to raw encoder units based on conversion factor.
	 * 
	 * @param inches
	 * @return
	 */
	public int inchesToEncUnits(double inches) {
		return (int) (inches * NumberConstants.kSwerveEncUnitsPerInch);
	}

	/**
	 * Convert velocity in raw units to inches/second.
	 * 
	 * @param encUnitsPer100ms
	 * @return
	 */
	public double encVelocityToInchesPerSecond(double encUnitsPer100ms) {
		return encUnitsToInches(encUnitsPer100ms) * 10;
	}

	/**
	 * Converts inches/sec to raw encoder units.
	 * 
	 * @param inchesPerSecond
	 * @return
	 */
	public int inchesPerSecondToEncVelocity(double inchesPerSecond) {
		return (int) (inchesToEncUnits(inchesPerSecond / 10.0));
	}

	/**
	 * Coverts degrees to raw encoder units.
	 * 
	 * @param degrees
	 * @return
	 */
	public int degreesToEncUnits(double degrees) {
		return (int) (degrees / 360.0 * NumberConstants.kSwerveDriveEncoderResolution);
	}

	/**
	 * Converts raw encoder units to degrees. (Assuming encoder is 1-1 with the module)
	 * 
	 * @param encUnits
	 * @return
	 */
	public double encUnitsToDegrees(double encUnits) {
		return encUnits / NumberConstants.kSwerveDriveEncoderResolution * 360.0;
	}

	/**
	 * @return the current robot position
	 */
	public Translation2d getPosition() {
		return position;
	}

	/**
	 * @return the estimated robot position
	 */
	public Pose2d getEstimatedRobotPose() {
		return estimatedRobotPose;
	}

	/**
	 * Update the pose of the module based off of module angle, 
	 * robot angle and distance driven.
	 * 
	 * @param robotHeading
	 */
	public synchronized void updatePose(Rotation2d robotHeading) {
		double currentEncDistance = getDriveDistanceInches();
		double deltaEncDistance = (currentEncDistance - previousEncDistance)
				* NumberConstants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(currentWheelAngle.cos() * deltaEncDistance,
				currentWheelAngle.sin() * deltaEncDistance);

		double xScrubFactor = NumberConstants.kXScrubFactor;
		double yScrubFactor = NumberConstants.kYScrubFactor;
		if (Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)) {
			if (standardCarpetDirection) {
				xScrubFactor = 1.0;
			} else {

			}
		} else {
			if (standardCarpetDirection) {

			} else {
				xScrubFactor = 1.0;
			}
		}
		if (Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)) {
			if (standardCarpetDirection) {
				yScrubFactor = 1.0;
			} else {

			}
		} else {
			if (standardCarpetDirection) {

			} else {
				yScrubFactor = 1.0;
			}
		}

		deltaPosition = new Translation2d(deltaPosition.x() * xScrubFactor, deltaPosition.y() * yScrubFactor);
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(startingPosition).inverse());
		position = updatedPosition;
		estimatedRobotPose = robotPose;
		previousEncDistance = currentEncDistance;
	}

	/**
	 * Reset the module position to a specified pose.
	 * 
	 * @param robotPose
	 */
	public synchronized void resetPose(Pose2d robotPose) {
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
		position = modulePosition;
	}

	/**
	 * Reset pose to the starting position.
	 */
	public synchronized void resetPose() {
		position = startingPosition;
	}

	public synchronized void resetLastEncoderReading() {
		previousEncDistance = getDriveDistanceInches();
	}

	@Override
	public void periodic() {
		periodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition(0);
		periodicIO.drivePosition = driveMotor.getSelectedSensorPosition(0);
		periodicIO.velocity = driveMotor.getSelectedSensorVelocity();
		
		rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
		driveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);

	}

	/**
	 * Stop the drive. 
	 */
	public synchronized void stop() {
		setDriveOpenLoop(0.0);
	}

	/**
	 * Stop the drive and rotation.
	 */
	public synchronized void disable() {
		setDriveOpenLoop(0.0);
		setRotationOpenLoop(0.0);
	}

	public synchronized void resetRotationToAbsolute() {
		rotationMotor.setSelectedSensorPosition(
				encoderReverseFactor * (rotationMotor.getSelectedSensorPosition() - encoderOffset), 0, 10);
	}

	public synchronized void zeroSensors() {
		zeroSensors(new Pose2d());
	}

	public synchronized void zeroSensors(Pose2d robotPose) {
		// driveMotor.setSelectedSensorPosition(0, 0, 100); TODO check if this is
		// necessary
		resetPose(robotPose);
		estimatedRobotPose = robotPose;
		previousEncDistance = getDriveDistanceInches();
	}

	/**
	 * Output swerve module telemetry to smartdahboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
		SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
		SmartDashboard.putNumber(name + "Pulse Width", rotationMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber(name + "Drive Voltage", periodicIO.driveVoltage);
		SmartDashboard.putNumber(name + "Rotation Voltage", rotationMotor.getMotorOutputVoltage());
		SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
		if (periodicIO.rotationControlMode == TalonFXControlMode.MotionMagic)
			SmartDashboard.putNumber(name + "Error", encUnitsToDegrees(rotationMotor.getClosedLoopError(0)));
		SmartDashboard.putNumber(name + "X", position.x());
		SmartDashboard.putNumber(name + "Y", position.y());
		SmartDashboard.putNumber(name + "Drive Current", driveMotor.getSupplyCurrent());
		SmartDashboard.putNumber(name + "Rotation Speed", rotationMotor.getSelectedSensorVelocity(0));
	}

	/**
	 * Class to contain the input and output variables for the module
	 */
	public static class PeriodicIO {
		// Inputs
		public int rotationPosition = 0;
		public int drivePosition = 0;
		public int velocity = 0;
		public double driveVoltage = 0.0;

		// Outputs
		public TalonFXControlMode rotationControlMode = TalonFXControlMode.PercentOutput;
		public TalonFXControlMode driveControlMode = TalonFXControlMode.PercentOutput;
		public double rotationDemand;
		public double driveDemand;
	}

}
