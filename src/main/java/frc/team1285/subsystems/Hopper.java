package frc.team1285.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1285.util.ElectricalConstants;

public class Hopper extends SubsystemBase {

    private CANSparkMax leftHopper;
    private CANSparkMax rightHopper;

    public Hopper() {
        leftHopper = new CANSparkMax(ElectricalConstants.LEFT_HOPPER, MotorType.kBrushless);
        rightHopper = new CANSparkMax(ElectricalConstants.RIGHT_HOPPER, MotorType.kBrushless);
        rightHopper.setInverted(true);
    }

    public void runLeftHopper(double output) {
        this.leftHopper.set(output);
    }

    public void runRightHopper(double output) {
        this.rightHopper.set(output);
    }

    public double getLeftVelocity() {
        return leftHopper.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightHopper.getEncoder().getVelocity();
    }

}