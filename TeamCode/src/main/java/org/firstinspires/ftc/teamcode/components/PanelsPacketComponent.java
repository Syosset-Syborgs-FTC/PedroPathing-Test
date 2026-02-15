package org.firstinspires.ftc.teamcode.components;

import com.bylazar.field.Style;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.SyborgsAutonBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.teleop.SyborgsTeleop;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

public class PanelsPacketComponent implements Component {
	public static final PanelsPacketComponent INSTANCE = new PanelsPacketComponent();
	@Override
	public void postUpdate() {
		Drawing.drawPoseHistory(follower().getPoseHistory());
		Drawing.drawRobot(follower().getPose());
		if (follower().getCurrentPathChain() != null) {
			Drawing.drawPath(follower().getCurrentPathChain(), new Style("#00FFFF", "#00FFFF", 0.75));
		}
		Drawing.sendPacket();
	}
	@Override
	public void postWaitForStart() {
		Drawing.sendPacket();
	}
}
