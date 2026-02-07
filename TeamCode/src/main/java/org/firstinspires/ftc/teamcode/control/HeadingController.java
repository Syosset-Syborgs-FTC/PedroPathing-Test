package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;

public class HeadingController {
	public static volatile double kP = 2;
	public static volatile double kI = 0.4;
	public static volatile double kD = 0.16;

	private final PIDFController headingPID;

	public HeadingController() {
		headingPID = new PIDFController(kP, kI, kD);
		headingPID.setContinuous(true, -Math.PI, Math.PI);
		headingPID.setOutputLimits(-1.0, 1.0);
	}

	public double getTurnPower(Pose pose, double targetX, double targetY) {
		headingPID.setConstants(kP, kI, kD);

		double currentHeading = pose.getHeading();

		double desiredHeading = Math.atan2(
				targetY - pose.getY(),
				targetX - pose.getX()
		);

		headingPID.setTarget(desiredHeading);

		return headingPID.update(currentHeading, 0);
	}

	public void reset() {
		headingPID.reset();
	}
}
