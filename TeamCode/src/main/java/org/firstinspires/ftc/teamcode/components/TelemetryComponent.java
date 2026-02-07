package org.firstinspires.ftc.teamcode.components;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class TelemetryComponent implements Component {
	public static final TelemetryComponent INSTANCE = new TelemetryComponent();
	@Override
	public void postUpdate() {
		ActiveOpMode.telemetry().update();
	}
	@Override
	public void postWaitForStart() {
		ActiveOpMode.telemetry().update();
	}
}
