package org.firstinspires.ftc.teamcode.components;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class LoopTimeCompenent implements Component {
	public static final LoopTimeCompenent INSTANCE = new LoopTimeCompenent();
	public double loopTime = 0;
	private double loopStart = 0;

	@Override
	public void preWaitForStart() {
		loopStart = ActiveOpMode.getRuntime();
	}
	@Override
	public void postWaitForStart() {
		loopTime = ActiveOpMode.getRuntime() - loopStart;
		ActiveOpMode.telemetry().addData("Loop Time (ms)", loopTime * 1000);
	}
	@Override
	public void preUpdate() {
		loopStart = ActiveOpMode.getRuntime();
	}
	@Override
	public void postUpdate() {
		loopTime = ActiveOpMode.getRuntime() - loopStart;
		ActiveOpMode.telemetry().addData("Loop Time (ms)", loopTime * 1000);
	}

}
