package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import dev.nextftc.core.components.Component;

public class PanelsPacketComponent implements Component {
	public static final PanelsPacketComponent INSTANCE = new PanelsPacketComponent();
	@Override
	public void postUpdate() {
		Drawing.sendPacket();
	}
	@Override
	public void postWaitForStart() {
		Drawing.sendPacket();
	}
}
