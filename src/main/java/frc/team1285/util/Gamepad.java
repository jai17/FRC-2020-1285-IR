package frc.team1285.util;

import edu.wpi.first.wpilibj.GenericHID;

public class Gamepad extends GenericHID {
    /******************************Joystick Mapping********************************/
    private final int X_BUTTON                        = 1;
    private final int A_BUTTON                        = 2;
    private final int B_BUTTON                        = 3;
    private final int Y_BUTTON                        = 4;
    private final int LEFT_BUMPER                     = 5;
    private final int RIGHT_BUMPER                    = 6;
    private final int LEFT_TRIGGER                    = 7;
    private final int RIGHT_TRIGGER                   = 8;
    private final int BACK_BUTTON                     = 9;
    private final int START_BUTTON                    = 10;
    private final int LEFT_ANALOG_BUTTON              = 11;
    private final int RIGHT_ANALOG_BUTTON             = 12;
    
    private final int LEFT_ANALOG_X                   = 0;
    private final int LEFT_ANALOG_Y                   = 1;
    private final int RIGHT_ANALOG_X                  = 2;
    private final int RIGHT_ANALOG_Y                  = 3;

    private final int DPAD_UP                         = 0;
    private final int DPAD_RIGHT                      = 90;
    private final int DPAD_DOWN                       = 180;
    private final int DPAD_LEFT                       = 270;

    /******************************Joystick Toggles********************************/

    ToggleBoolean X_BUTTON_TOGGLE;
    ToggleBoolean A_BUTTON_TOGGLE;
    ToggleBoolean B_BUTTON_TOGGLE;
    ToggleBoolean Y_BUTTON_TOGGLE;
    ToggleBoolean LEFT_BUMPER_TOGGLE;
    ToggleBoolean RIGHT_BUMPER_TOGGLE;
    ToggleBoolean LEFT_TRIGGER_TOGGLE;
    ToggleBoolean RIGHT_TRIGGER_TOGGLE;
    ToggleBoolean BACK_BUTTON_TOGGLE;
    ToggleBoolean START_BUTTON_TOGGLE;
    ToggleBoolean DPAD_UP_TOGGLE;
    ToggleBoolean DPAD_RIGHT_TOGGLE;
    ToggleBoolean DPAD_DOWN_TOGGLE;
    ToggleBoolean DPAD_LEFT_TOGGLE;


    public Gamepad(int portID) {
        super(portID);

        X_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        A_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        B_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        Y_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        LEFT_BUMPER_TOGGLE = new ToggleBoolean(0.5);
        RIGHT_BUMPER_TOGGLE = new ToggleBoolean(0.5);
        LEFT_TRIGGER_TOGGLE = new ToggleBoolean(0.5);
        RIGHT_TRIGGER_TOGGLE = new ToggleBoolean(0.5);
        BACK_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        START_BUTTON_TOGGLE = new ToggleBoolean(0.5);
        DPAD_UP_TOGGLE = new ToggleBoolean(0.5);
        DPAD_RIGHT_TOGGLE = new ToggleBoolean(0.5);
        DPAD_DOWN_TOGGLE = new ToggleBoolean(0.5);
        DPAD_LEFT_TOGGLE = new ToggleBoolean(0.5);
    }

    /**
     * Used to return the controller's left joystick x-axis value
     * 
     * @return Returns x-value from left joystick
     */
    public double getLeftX() {
        double joy = super.getRawAxis(LEFT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the controller's left joystick y-axis value
     * 
     * @return Returns y-value from left joystick
     */
    public double getLeftY() {
        double joy = super.getRawAxis(LEFT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the controller's right joystick x-axis value
     * 
     * @return Returns x-value from right joystick
     */
    public double getRightX() {
        double joy = super.getRawAxis(RIGHT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the controller's right joystick y-axis value
     * 
     * @return Returns y-value from right joystick
     */
    public double getRightY() {
        double joy = super.getRawAxis(RIGHT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    
    /**
     * Checks if the D PAD was pressed in the up direction
     * 
     * @return true if pressed in the up direction
     */
    public boolean getDPadUp() {
        if (super.getPOV(0) == DPAD_UP) {
            DPAD_UP_TOGGLE.set();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Checks the D PAD up toggle
     * 
     * @return true if toggle set in the up direction
     */
    public boolean getDPadUpToggle() {
        getDPadUp();
        return DPAD_UP_TOGGLE.get();
    }

    /**
     * Checks if the D PAD was pressed in the right direction
     * 
     * @return true if pressed in the right direction
     */
    public boolean getDPadRight() {
        if (super.getPOV(0) == DPAD_RIGHT) {
            DPAD_RIGHT_TOGGLE.set();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Checks the D PAD Right toggle
     * 
     * @return true if toggle set in the right direction
     */
    public boolean getDPadRightToggle() {
        getDPadRight();
        return DPAD_RIGHT_TOGGLE.get();
    }

    /**
     * Checks if the D PAD was pressed in the down direction
     * 
     * @return true if pressed in the down direction
     */
    public boolean getDPadDown() {
        if (super.getPOV(0) == DPAD_DOWN) {
            DPAD_DOWN_TOGGLE.set();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Checks the D PAD Down toggle
     * 
     * @return true if toggle set in the down direction
     */
    public boolean getDPadDownToggle() {
        getDPadDown();
        return DPAD_DOWN_TOGGLE.get();
    }

    /**
     * Checks if the D PAD was pressed in the left direction
     * 
     * @return true if pressed in the left direction
     */
    public boolean getDPadLeft() {
        if (super.getPOV(0) == DPAD_LEFT) {
            DPAD_LEFT_TOGGLE.set();
            return true;
        } else {
            return false;
        }
    }

    /**
     * Checks the D PAD left toggle
     * 
     * @return true if toggle set in the left direction
     */
    public boolean getDPadLeftToggle() {
        getDPadLeft();
        return DPAD_LEFT_TOGGLE.get();
    }

    
    /**
     * Used to return the controller's left trigger value
     * 
     * @return Returns value from left joystick
     */
    public boolean getLeftTrigger() {
        if(super.getRawButton(LEFT_TRIGGER))
            LEFT_TRIGGER_TOGGLE.set();
        return super.getRawButton(LEFT_TRIGGER);
    }

    public boolean getLeftTriggerPressed() {
        return super.getRawButtonPressed(LEFT_TRIGGER);
    }

    public boolean getLeftTriggerReleased() {
        return super.getRawButtonReleased(LEFT_TRIGGER);
    }

    /**
     * Checks the Left Trigger toggle
     * 
     * @return true if toggle set for left trigger
     */
    public boolean getLeftTriggerToggle() {
        getLeftTrigger();
        return LEFT_TRIGGER_TOGGLE.get();
    }

    /**
     * Used to return the controller's right trigger value
     * 
     * @return Returns value from right trigger
     */
    public boolean getRightTrigger() {
        if(super.getRawButton(RIGHT_TRIGGER))
            RIGHT_TRIGGER_TOGGLE.set();
        return super.getRawButton(RIGHT_TRIGGER);
    }

    public boolean getRightTriggerPressed() {
        return super.getRawButtonPressed(RIGHT_TRIGGER);
    }

    public boolean getRightTriggerReleased() {
        return super.getRawButtonReleased(RIGHT_TRIGGER);
    }

    /**
     * Checks the Right Trigger toggle
     * 
     * @return true if toggle set for right trigger
     */
    public boolean getRightTriggerToggle() {
        getRightTrigger();
        return RIGHT_TRIGGER_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getLeftBumper() {
        if(super.getRawButton(LEFT_BUMPER))
            LEFT_BUMPER_TOGGLE.set();
        return super.getRawButton(LEFT_BUMPER);
    }

    public boolean getLeftBumperPressed() {
        return super.getRawButtonPressed(LEFT_BUMPER);
    }

    public boolean getLeftBumperReleased() {
        return super.getRawButtonReleased(LEFT_BUMPER);
    }

    /**
     * Checks the Left Bumper toggle
     * 
     * @return true if toggle set for left bumper
     */
    public boolean getLeftBumperToggle() {
        getLeftBumper();
        return LEFT_BUMPER_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getRightBumper() {
        if(super.getRawButton(RIGHT_BUMPER))
            RIGHT_BUMPER_TOGGLE.set();
        return super.getRawButton(RIGHT_BUMPER);
    }

    public boolean getRightBumperPressed() {
        return super.getRawButtonPressed(RIGHT_BUMPER);
    }

    public boolean getRightBumperReleased() {
        return super.getRawButtonReleased(RIGHT_BUMPER);
    }

    /**
     * Checks the Right Bumper toggle
     * 
     * @return true if toggle set for right bumper
     */
    public boolean getRightBumperToggle() {
        getRightBumper();
        return RIGHT_BUMPER_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getAButton() {
        if(super.getRawButton(A_BUTTON))
            A_BUTTON_TOGGLE.set();
        return super.getRawButton(A_BUTTON);
    }

    public boolean getAButtonPressed() {
        return super.getRawButtonPressed(A_BUTTON);
    }

    public boolean getAButtonReleased() {
        return super.getRawButtonReleased(A_BUTTON);
    }

    /**
     * Checks the A button toggle
     * 
     * @return true if toggle set for the A button
     */
    public boolean getAButtonToggle() {
        getAButton();
        return A_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getBButton() {
        if(super.getRawButton(B_BUTTON))
            B_BUTTON_TOGGLE.set();
        return super.getRawButton(B_BUTTON);
    }

    public boolean getBButtonPressed() {
        return super.getRawButtonPressed(B_BUTTON);
    }

    public boolean getBButtonReleased() {
        return super.getRawButtonReleased(B_BUTTON);
    }

    /**
     * Checks the B button toggle
     * 
     * @return true if toggle set for the B button
     */
    public boolean getBButtonToggle() {
        getBButton();
        return B_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getXButton() {
        if(super.getRawButton(X_BUTTON))
            X_BUTTON_TOGGLE.set();
        return super.getRawButton(X_BUTTON);
    }

    public boolean getXButtonPressed() {
        return super.getRawButtonPressed(X_BUTTON);
    }

    public boolean getXButtonReleased() {
        return super.getRawButtonReleased(X_BUTTON);
    }

    /**
     * Checks the X button toggle
     * 
     * @return true if toggle set for the X button
     */
    public boolean getXButtonToggle() {
        getXButton();
        return X_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getYButton() {
        if(super.getRawButton(Y_BUTTON))
            Y_BUTTON_TOGGLE.set();
        return super.getRawButton(Y_BUTTON);
    }

    public boolean getYButtonPressed() {
        return super.getRawButtonPressed(Y_BUTTON);
    }

    public boolean getYButtonReleased() {
        return super.getRawButtonReleased(Y_BUTTON);
    }

    /**
     * Checks the Y button toggle
     * 
     * @return true if toggle set for the Y button
     */
    public boolean getYButtonToggle() {
        getYButton();
        return Y_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getStartButton() {
        if(super.getRawButton(START_BUTTON))
            START_BUTTON_TOGGLE.set();
        return super.getRawButton(START_BUTTON);
    }

    public boolean getStartButtonPressed() {
        return super.getRawButtonPressed(START_BUTTON);
    }

    public boolean getStartButtonReleased() {
        return super.getRawButtonReleased(START_BUTTON);
    }

    /**
     * Checks the Start button toggle
     * 
     * @return true if toggle set for the Start button
     */
    public boolean getStartButtonToggle() {
        getStartButton();
        return START_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getBackButton() {
        if(super.getRawButton(BACK_BUTTON))
            BACK_BUTTON_TOGGLE.set();
        return super.getRawButton(BACK_BUTTON);
    }

    public boolean getBackButtonPressed() {
        return super.getRawButtonPressed(BACK_BUTTON);
    }

    public boolean getBackButtonReleased() {
        return super.getRawButtonReleased(BACK_BUTTON);
    }

    /**
     * Checks the Back button toggle
     * 
     * @return true if toggle set for the Start button
     */
    public boolean getBackButtonToggle() {
        getBackButton();
        return BACK_BUTTON_TOGGLE.get();
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getLeftAnalogButton() {
        return super.getRawButton(LEFT_ANALOG_BUTTON);
    }

    public boolean getLeftAnalogButtonPressed() {
        return super.getRawButtonPressed(LEFT_ANALOG_BUTTON);
    }

    public boolean getLeftAnalogButtonReleased() {
        return super.getRawButtonReleased(LEFT_ANALOG_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getRightAnalogButton() {
        return super.getRawButton(RIGHT_ANALOG_BUTTON);
    }

    public boolean getRightAnalogButtonPressed() {
        return super.getRawButtonPressed(RIGHT_ANALOG_BUTTON);
    }

    public boolean getRightAnalogButtonReleased() {
        return super.getRawButtonReleased(RIGHT_ANALOG_BUTTON);
    }

    @Override
    public double getX(Hand hand) {
        if (hand == Hand.kLeft) {
            return this.getLeftX();
        }
        else {
            return this.getRightX();
        }
    }

    @Override
    public double getY(Hand hand) {
        if (hand == Hand.kLeft) {
            return this.getLeftY();
        }
        else {
            return this.getRightY();
        }
    }
}