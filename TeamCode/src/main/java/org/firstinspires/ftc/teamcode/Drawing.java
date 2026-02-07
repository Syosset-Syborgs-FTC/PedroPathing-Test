package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.geometry.Pose;

import com.pedropathing.math.Vector;
public final class Drawing {
	private Drawing() {}


	public static void drawRobot(Canvas c, Pose t) {
		final double ROBOT_RADIUS = 9;

		c.setStrokeWidth(1);
		c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

		Vector halfv = t.getHeadingAsUnitVector().times(0.5 * ROBOT_RADIUS);
		Vector p1 = t.getAsVector().plus(halfv);
		Vector p2 = p1.plus(halfv);
		c.strokeLine(p1.getXComponent(), p1.getYComponent(), p2.getXComponent(), p2.getYComponent());
	}
}
