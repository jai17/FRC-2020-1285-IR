package frc.team1285.loops;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team1285.subsystems.Intake;
import frc.team1285.util.OI;

public class IntakeControl extends Intake {

    private static IntakeControl mInstance;

    private double speedFactor = 0.85;

    // Auto
    private double aIntakeSpeed = 0.0;

    private OI oi;

    public static IntakeControl getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeControl();
        }
        return mInstance;
    }

    private IntakeControl() {
        super();
        oi = OI.getInstance();
    }

    public enum IntakeControlState {
        OPEN_LOOP, // open loop voltage
        AUTO_LOOP
    }

    private IntakeControlState mControlState = IntakeControlState.OPEN_LOOP;

    @Override
    public void periodic() {
        if (DriverStation.getInstance().isAutonomous()) {
            mControlState = IntakeControlState.AUTO_LOOP;
        }
        switch (mControlState) {
            case OPEN_LOOP:
                if (this.oi.toolPad.getAButton()) {
                    super.runIntake(speedFactor);
                } else if (this.oi.toolPad.getBButton()) {
                    super.runIntake(-speedFactor);
                } else if (this.oi.toolPad.getLeftTrigger()) {
                    super.runIntake(0.4);
                } else {
                    super.runIntake(0);
                }

                if (this.oi.toolPad.getDPadLeftToggle()) {
                    super.extendIntake();
                } else {
                    super.retractIntake();
                }
                break;
            case AUTO_LOOP:
                super.runIntake(aIntakeSpeed);
                break;
        }
    }

    // Set the intake control state
    public void setControlState(IntakeControlState mControlState) {
        this.mControlState = mControlState;
    }

    // Get the current intake state
    public IntakeControlState getControlState() {
        return mControlState;
    }

    // Set the intake speed
    public void setIntakeSpeed(double speed) {
        this.aIntakeSpeed = speed;
    }
}
