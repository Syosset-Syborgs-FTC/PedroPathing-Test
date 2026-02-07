package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Common {
	public enum Alliance {
		Red,
		Blue;

		public Alliance getOpposite() {
			return this == Red ? Blue : Red;
		}
	}
	public static Alliance alliance = Alliance.Red;
	public static Telemetry telemetry = null;
	public static Pose applyTransform(Pose base, Pose offset) {
		Pose rotatedOffset = offset.rotate(base.getHeading(), false);

		return new Pose(
				base.getX() + rotatedOffset.getX(),
				base.getY() + rotatedOffset.getY(),
				MathFunctions.normalizeAngle(base.getHeading() + offset.getHeading()),
				base.getCoordinateSystem()
		);
	}

	public static Pose inverse(Pose pose) {
		double invHeading = -pose.getHeading();

		Pose inverted = new Pose(-pose.getX(), -pose.getY(), invHeading, pose.getCoordinateSystem())
				.rotate(invHeading, false);

		return inverted.withHeading(MathFunctions.normalizeAngle(invHeading));
	}
}
