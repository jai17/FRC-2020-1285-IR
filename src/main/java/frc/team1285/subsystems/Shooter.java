package frc.team1285.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1285.util.ElectricalConstants;
import frc.team1285.util.NumberConstants;

public class Shooter extends SubsystemBase {
    // Motor Controllers
    private TalonFX leftShooter;
    private TalonFX rightShooter;
    private TalonSRX conveyorMotor;

    public Shooter() {

        // rightShooter Master Initialisation
        rightShooter = new TalonFX(ElectricalConstants.RIGHT_SHOOTER);
        rightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        rightShooter.setInverted(false);
        rightShooter.setSensorPhase(false);
        rightShooter.setNeutralMode(NeutralMode.Coast);

        // leftShooter Initialisation
        leftShooter = new TalonFX(ElectricalConstants.LEFT_SHOOTER);
        leftShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        leftShooter.setInverted(true);
        leftShooter.setSensorPhase(false);
        leftShooter.setNeutralMode(NeutralMode.Coast);
        leftShooter.follow(rightShooter);

        // conveyorMotor Intialisation
        conveyorMotor = new TalonSRX(ElectricalConstants.CONVEYOR_MOTOR);

        // Method in order to set a default Motion Magic Velocity and Acceleration
        rightShooter.configMotionAcceleration(2500, 0);
        rightShooter.configMotionCruiseVelocity(NumberConstants.SHOOTER_MAX_SPEED, 0);

        rightShooter.config_kP(0, NumberConstants.pTalonShooter, 0);
        rightShooter.config_kI(0, NumberConstants.iTalonShooter, 0);
        rightShooter.config_kD(0, NumberConstants.dTalonShooter, 0);
        rightShooter.config_kF(0, NumberConstants.fTalonShooter, 10);
        rightShooter.selectProfileSlot(0, 0);
    }

    public void runShooter(double output) {
        this.rightShooter.set(TalonFXControlMode.PercentOutput, output);
    }

    public void runConveyor(double output) {
        this.conveyorMotor.set(ControlMode.PercentOutput, output);
    }

    public double getOutput() {
        return leftShooter.getMotorOutputPercent();
    }

    public double getShooterEncoder() {
        return leftShooter.getSelectedSensorPosition(0) * NumberConstants.SHOOTER_ENCODER_DIST_PER_TICK;
    }

    public double getShooterRotations() {
        return leftShooter.getSelectedSensorPosition(0);
    }

    public double getShooterSpeed() {
        return leftShooter.getSelectedSensorVelocity(0) * 10;
    }

    public double getRPM() {
        return getShooterSpeed() / 2048.0 * 60.0;
    }

    public double getRightShooterSpeed() {
        return rightShooter.getSelectedSensorVelocity(0) * 10;
    }

    public double getRightRPM() {
        return getRightShooterSpeed() / NumberConstants.TALONFX_PULSE_PER_ROTATION * 60.0;
    }

    public double RPMToTicks(double rpm) {
        return rpm / 600.0 * NumberConstants.TALONFX_PULSE_PER_ROTATION;
    }

    public void resetEncoders() {
        rightShooter.setSelectedSensorPosition(0, 0, 0);
    }

    public void setRPM(double rpm) {
        rightShooter.configClosedloopRamp(0);
        rightShooter.set(TalonFXControlMode.Velocity, RPMToTicks(rpm));
    }

    public void setPIDS(double p, double i, double d) {
        rightShooter.config_kP(0, p, 0);
        rightShooter.config_kI(0, i, 0);
        rightShooter.config_kD(0, d, 0);
    }
}