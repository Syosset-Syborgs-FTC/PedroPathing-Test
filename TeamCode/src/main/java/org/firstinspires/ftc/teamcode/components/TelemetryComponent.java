package org.firstinspires.ftc.teamcode.components;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class TelemetryComponent implements Component {
	public static final TelemetryComponent INSTANCE = new TelemetryComponent();
	@Override
	public void preInit() {
		Telemetry originalTelemetry = ActiveOpMode.telemetry();
		Objects.requireNonNull(ActiveOpMode.it).telemetry = new JoinedTelemetry(originalTelemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
	}
	@Override
	public void postUpdate() {
		ActiveOpMode.telemetry().update();
	}
	@Override
	public void postWaitForStart() {
		ActiveOpMode.telemetry().update();
	}
}
