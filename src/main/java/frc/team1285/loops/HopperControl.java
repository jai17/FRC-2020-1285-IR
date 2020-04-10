package frc.team1285.loops;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team1285.subsystems.Hopper;
import frc.team1285.util.OI;

public class HopperControl extends Hopper {

    private static HopperControl mInstance;

    private OI oi;

    private double speed = 0.6;

    // Auto
    private double aLeftHopperSpeed = 0.0;
    private double aRightHopperSpeed = 0.0;

    public static HopperControl getInstance() {
        if (mInstance == null) {
            mInstance = new HopperControl();
        }
        return mInstance;
    }

    private HopperControl() {
        oi = OI.getInstance();
    }

    public enum HopperControlState {
        OPEN_LOOP, AUTO_LOOP
    }

    private HopperControlState mControlState = HopperControlState.OPEN_LOOP;

    @Override
    public void periodic() {
        if (DriverStation.getInstance().isAutonomous()) {
            mControlState = HopperControlState.AUTO_LOOP;
        }
        switch (mControlState) {
            case OPEN_LOOP:
                if (this.oi.toolPad.getXButton()) {
                    super.runLeftHopper(-speed);
                    super.runRightHopper(speed);
                } else if (this.oi.toolPad.getYButton()) {
                    super.runLeftHopper(speed);
                    super.runRightHopper(-speed);
                } else if (this.oi.toolPad.getLeftTrigger()) {
                    super.runLeftHopper(-speed * 1.0);
                    super.runRightHopper(speed * 0.7);
                } else {
                    super.runLeftHopper(0);
                    super.runRightHopper(0);
                }
                break;
            case AUTO_LOOP:
                super.runLeftHopper(aLeftHopperSpeed);
                super.runRightHopper(aRightHopperSpeed);
                break;
        }
    }

    public void setControlState(HopperControlState mControlState) {
        this.mControlState = mControlState;
    }

    public HopperControlState getControlState() {
        return mControlState;
    }

    // Variable Setters and Getters

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return this.speed;
    }

    // Auto Speed Setters
    public void setLeftHopperSpeed(double speed) {
        this.aLeftHopperSpeed = speed;
    }

    public void setRightHopperSpeed(double speed) {
        this.aRightHopperSpeed = speed;
    }

}