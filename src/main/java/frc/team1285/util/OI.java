package frc.team1285.util;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Attributes
    private static OI mInstance;

    private static final int DRIVE_USB_PORT = 0;
    private static final int TOOL_USB_PORT = 1;

    public Gamepad drivePad;
    public Gamepad toolPad;

    /**
     * Initializes the joystick objects
     */
    public OI() {
        drivePad = new Gamepad(DRIVE_USB_PORT);
        toolPad = new Gamepad(TOOL_USB_PORT);
    }

    public static OI getInstance() {
        if (mInstance == null) {
            mInstance = new OI();
        }
        return mInstance;
    }
}