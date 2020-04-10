package frc.team1285.loops;

import edu.wpi.first.wpilibj.Preferences;
import frc.team1285.subsystems.Shooter;
import frc.team1285.util.OI;

public class ShooterControl extends Shooter {

    private static ShooterControl mInstance;

    private double conveyorSpeeed = 0.91;

    private OI oi;

    // Auto constants
    private double rpm = 0.0;
    private double aConveyorSpeed = 0.0;

    Preferences prefs;

    public static ShooterControl getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterControl();
        }
        return mInstance;
    }

    private ShooterControl() {
        super();
        oi = OI.getInstance();
        prefs = Preferences.getInstance();
    }

    public enum ShooterControlState {
        OPEN_LOOP, AUTO_LOOP
    }

    private ShooterControlState mControlState = ShooterControlState.OPEN_LOOP;

    @Override
    public void periodic() {
        switch (mControlState) {
            case OPEN_LOOP:
                super.setRPM(rpm);
                if (this.oi.toolPad.getLeftTrigger()) {
                    super.runConveyor(conveyorSpeeed);
                } else if (this.oi.toolPad.getAButton()) {
                    super.runConveyor(-0.25);
                } else {
                    super.runConveyor(0);
                }
                break;

            case AUTO_LOOP:
                if (rpm > 0) {
                    super.setRPM(rpm);
                } else {
                    runShooter(0);
                }

                super.runConveyor(aConveyorSpeed);
                break;
        }
    }

    // Set the shooter control state
    public void setControlState(ShooterControlState mControlState) {
        this.mControlState = mControlState;
    }

    // Get the current shooter state
    public ShooterControlState getControlState() {
        return mControlState;
    }

    public void setRPM(double rpm) {
        this.rpm = rpm;
    }

    public void setConveyorSpeed(double speed) {
        this.aConveyorSpeed = speed;
    }

}