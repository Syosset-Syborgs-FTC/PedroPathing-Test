package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.OptionalInt;

public class Common {
	@NonNull
	@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
	public static String formatObeliskID(OptionalInt x) {
		if (x.isPresent()) {
			int xValue = x.getAsInt();
			switch (xValue) {
				case 21:
					return "GPP";
				case 22:
					return "PGP";
				case 23:
					return "PPG";
				default:
					return Integer.toString(xValue);
			}
		}
		return "No obelisk apriltag visible";
	}

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
