package frc.team1285.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1285.util.ElectricalConstants;

public class Intake extends SubsystemBase {
    private TalonSRX intakeMotor;
    private DoubleSolenoid intakeSolenoid;

    public Intake() {
        intakeMotor = new TalonSRX(ElectricalConstants.INTAKE_MOTOR);

        intakeSolenoid = new DoubleSolenoid(ElectricalConstants.INTAKE_A, ElectricalConstants.INTAKE_B);
    }

    public void runIntake(double output) {
        this.intakeMotor.set(ControlMode.PercentOutput, output);
    }

    public void extendIntake() {
        this.intakeSolenoid.set(Value.kForward);
    }

    public void retractIntake() {
        this.intakeSolenoid.set(Value.kReverse);

    }

}
