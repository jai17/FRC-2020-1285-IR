package frc.team254.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import frc.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {

	private PigeonIMU pigeon;

	public Pigeon(TalonSRX talon) {
		try {
			pigeon = new PigeonIMU(talon);
		} catch (Exception e) {
			System.out.println(e);
		}
	}

	public Pigeon(int pigeonID) {
		try {
			pigeon = new PigeonIMU(pigeonID);
		} catch (Exception e) {
			System.out.println(e);
		}
	}

	public boolean isGood() {
		return (pigeon.getState() == PigeonState.Ready) ? true : false;
	}

	public Rotation2d getYaw() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		SmartDashboard.putNumber("Pigeon Heading", -pigeon.getFusedHeading(fusionStatus));
		return Rotation2d.fromDegrees(-pigeon.getFusedHeading(fusionStatus)/*-ypr[0]*/);
	}

	public double getPitch() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

	public void setAngle(double angle) {
		pigeon.setFusedHeading(-angle * 64.0, 10);
		pigeon.setYaw(-angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}

}
