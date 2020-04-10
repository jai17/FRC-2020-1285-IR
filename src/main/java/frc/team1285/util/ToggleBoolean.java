package frc.team1285.util;

import edu.wpi.first.wpilibj.Timer;

public class ToggleBoolean {
	private boolean toggle = false;
	private double waitTime = 0;

	Timer timer = new Timer();
	public boolean get;

	public ToggleBoolean() {
		waitTime = 0.5;
	}

	public ToggleBoolean(double waitTime) {
		this.waitTime = waitTime;
		timer.start();
	}

	public void set() {
		if (timer.get() > waitTime) {
			toggle = !toggle;
		}
		if (timer.get() > waitTime) {
			timer.reset();
			timer.start();
		}
	}

	public boolean get() {
		return toggle;
	}

	public void setTime(double waitTime) {
		this.waitTime = waitTime;
	}
}