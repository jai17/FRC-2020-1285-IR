package frc.team1285.loops;

import edu.wpi.first.wpilibj.Timer;
import frc.team1285.subsystems.Climber;
import frc.team1285.util.OI;

public class ClimberControl extends Climber {

    private static ClimberControl mInstance;

    private OI oi;

    Timer unLatchedTimer;
    Timer latchingTimer;
    Timer grabberTimer;
    public boolean latchingTimerStarted = false;

    public static ClimberControl getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberControl();
        }
        return mInstance;
    }

    private ClimberControl() {
        super();
        oi = OI.getInstance();
        unLatchedTimer = new Timer();
        latchingTimer = new Timer();
        grabberTimer = new Timer();
    }

    public enum ClimberControlState {
        OPEN_LOOP, // open loop voltage
        AUTO_LOOP, CLIMB_LOOP
    }

    private ClimberControlState mControlState = ClimberControlState.OPEN_LOOP;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (mControlState) {
            case OPEN_LOOP:
                if (oi.drivePad.getDPadLeft()) {
                    super.unlatchClimber();
                    unLatchedTimer.reset();
                    unLatchedTimer.start();
                    mControlState = ClimberControlState.CLIMB_LOOP;
                }
                break;
            case CLIMB_LOOP:

                if (super.getLeftClimberSwitch() && super.getRightClimberSwitch() && !latchingTimerStarted
                        && unLatchedTimer.get() > 1) {
                    latchingTimer.reset();
                    latchingTimer.start();
                    latchingTimerStarted = true;
                }
                if (latchingTimer.get() > 0 && latchingTimerStarted && unLatchedTimer.get() > 3) {
                    super.latchClimber();
                    // mControlState = ClimberControlState.OPEN_LOOP;
                    latchingTimerStarted = false;
                }
                break;
            case AUTO_LOOP:
                break;
        }
    }

    public void setControlState(ClimberControlState state) {
        this.mControlState = state;
    }

    public ClimberControlState getControlState() {
        return this.mControlState;
    }
}
