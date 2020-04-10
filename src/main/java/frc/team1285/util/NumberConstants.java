package frc.team1285.util;

import java.util.Arrays;
import java.util.List;

import frc.team1285.util.kinematics.InterpolatingDouble;
import frc.team1285.util.kinematics.InterpolatingTreeMap;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

/*
    Class that holds number constants for the robot 
    - PID Controller gains
    - Subsystem constraints
    - Path following constants
*/
public class NumberConstants {
    // **************************************************************************
    // *************************** ENCODER CONSTANTS ****************************
    // **************************************************************************
    public static final double kEpsilon = 0.0001;
    public static final double TALONFX_PULSE_PER_ROTATION = 2048.0;
    public static final int MAG_PULSE_PER_ROTATION = 4096; // encoder pulse per rotation

    // **************************************************************************
    // ********************* SHOOTER PID CONSTANTS ****************************
    // **************************************************************************

    public static final double SHOOTER_MAX_ACCELERATION = 6000 * 2096;
    public static final int SHOOTER_MAX_SPEED = 6000 * 2096;
    public static final double pTalonShooter = 0.13;
    public static final double iTalonShooter = 0;
    public static final double dTalonShooter = 10;
    public static final double fTalonShooter = 0.65 * 1023.0 / 13900.0;

    public static final double SHOOTER_ENCODER_DIST_PER_TICK = 4 * Math.PI / 2096;

    // **************************************************************************
    // ****************************** AUTO CONSTANTS ****************************
    // **************************************************************************

    public static final double BATTERED_SHOT_RPM = 3800;

    // **************************************************************************
    // ****************************** SWERVE CONSTANTS **************************
    // **************************************************************************

    public static final double kSwerveDriveMaxSpeed = 28000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
    public static final double kSwerveRotationMaxSpeed = 1250.0 * 0.8; // The 0.8 is to request a speed that is always
                                                                       // achievable
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;

    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth = 21.0;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);

    // Swerve Odometry Constants
    public static final double kSwerveWheelDiameter = 4.0901; // inches (actual diamter is closer to 3.87, but secondary
                                                              // algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 4096.0;
    /**
     * The number of rotations the swerve drive encoder undergoes for every rotation
     * of the wheel.
     */
    public static final double kSwerveEncoderToWheelRatio = 6.0;
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

    // Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.5;
    public static final double kRobotLength = 36.5;
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    public static final double kRobotProbeExtrusion = 4.0;

    // Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[] { 1.0, 1.0, 1.0, 1.0 };
    public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));

    // Swerve Module Wheel Offsets (Rotation encoder values when the wheels are
    // facing 0 degrees)
    public static final int kFrontRightEncoderStartingPos = -1403 - 1024;
    public static final int kFrontLeftEncoderStartingPos = -2171 - 1024;
    public static final int kRearLeftEncoderStartingPos = -1327 - 1024;
    public static final int kRearRightEncoderStartingPos = -5953 - 1024;

    // Swerve Module Positions (relative to the center of the drive base)
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2,
            kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength / 2,
            -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2,
            -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength / 2,
            kWheelbaseWidth / 2);

    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero, kVehicleToModuleOne,
            kVehicleToModuleTwo, kVehicleToModuleThree);

    // **************************************************************************
    // ****************************** SWERVE PID CONSTANTS **********************
    // **************************************************************************

    public static final double SWERVE_P = 0.018;
    public static final double SWERVE_I = 0.1;
    public static final double SWERVE_D = 0.06;
    public static final double SWERVE_TOLERANCE = 5.0;

    public static final double SWERVE_VEL_P = 0.018;
    public static final double SWERVE_VEL_I = 0.1;
    public static final double SWERVE_VEL_D = 0.06;
    public static final double SWERVE_VEL_TOLERANCE = 5.0;

    public static final double SWERVE_ROTATION_P = 0.035;
    public static final double SWERVE_ROTATION_I = 0;
    public static final double SWERVE_ROTATION_D = 0.35;
    public static final double SWERVE_ROTATION_TOLERANCE = 1.0;

    // Swerve Speed Constraint Treemap (i.e based on elevator height)
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSwerveSpeedTreeMap = new InterpolatingTreeMap<>();
    static {
        kSwerveSpeedTreeMap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(1.0));
        kSwerveSpeedTreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1.0));
        kSwerveSpeedTreeMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(1.0));
        kSwerveSpeedTreeMap.put(new InterpolatingDouble(10.0), new InterpolatingDouble(0.5));
        kSwerveSpeedTreeMap.put(new InterpolatingDouble(15 + 2.0), new InterpolatingDouble(0.5));
    }

    // **************************************************************************
    // ************************** PATH FOLLOWING CONSTANTS **********************
    // **************************************************************************

    public static final double kPathLookaheadTime = 0.25; // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0; // inches 24.0 (we've been using 3.0)

    // Field Landmarks
    public static final Pose2d kRobotLeftStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
    public static final Pose2d kRobotRightStartingPose = new Pose2d(
            new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));

    public static final Pose2d closeShipPosition = new Pose2d(new Translation2d(260.8, -28.87),
            Rotation2d.fromDegrees(90.0));
    public static final Pose2d closeHatchPosition = new Pose2d(new Translation2d(48.0 + 166.57, 27.44 - 10.0 - 162.0),
            Rotation2d.fromDegrees(-30.0));
    public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72 - 162.0),
            Rotation2d.fromDegrees(0.0));
    public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44 - 162.0),
            Rotation2d.fromDegrees(-90.0));

}