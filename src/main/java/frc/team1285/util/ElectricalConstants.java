package frc.team1285.util;

/*
    Class to hold electrical constants of the robot
    - Motor IDs
    - Solenoid IDs
    - Encoder IDs
    - Encoder Conversion Constants
*/
public class ElectricalConstants {

    // Gyro Constant
    public static final int PIGEON_IMU = 13;

    // **************************************************************************
    // ****************************** Swerve CONSTANTS **************************
    // **************************************************************************

    public static final int FRONT_RIGHT_ROTATION = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int FRONT_RIGHT_ROTATION_ENCODER = 3;

    public static final int FRONT_LEFT_ROTATION = 4;
    public static final int FRONT_LEFT_DRIVE = 5;
    public static final int FRONT_LEFT_ROTATION_ENCODER = 6;

    public static final int REAR_RIGHT_ROTATION = 7;
    public static final int REAR_RIGHT_DRIVE = 8;
    public static final int REAR_RIGHT_ROTATION_ENCODER = 9;

    public static final int REAR_LEFT_ROTATION = 10;
    public static final int REAR_LEFT_DRIVE = 11;
    public static final int REAR_LEFT_ROTATION_ENCODER = 12;

    // **************************************************************************
    // ******************************* INTAKE ***********************************
    // **************************************************************************

    public static final int INTAKE_MOTOR = 7;

    // **************************************************************************
    // ******************************* CLIMBER ***********************************
    // ***************************************************************************

    // Solenoids
    public static final int CLIMBER_A = 1;
    public static final int CLIMBER_B = 0;
    public static final int INTAKE_A = 7;
    public static final int INTAKE_B = 6;

    /// Digital Inputs
    public static final int LEFT_CLIMBER_SENSOR = 1;
    public static final int RIGHT_CLIMBER_SENSOR = 0;

    // **************************************************************************wq
    // ******************************* SHOOTER ********************************
    // **************************************************************************

    // Motors
    public static final int RIGHT_SHOOTER = 5;
    public static final int LEFT_SHOOTER = 6;
    public static final int CONVEYOR_MOTOR = 10;

    // **************************************************************************
    // ****************************** HOPPER ********************************
    // **************************************************************************

    public static final int LEFT_HOPPER = 8;
    public static final int RIGHT_HOPPER = 9;
}