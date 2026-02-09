package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Common.formatObeliskID;
import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawRobot;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.components.LoopTimeCompenent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.RGBFlywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class SyborgsAutonBase extends NextFTCOpMode {
	public static FuturePose currentPose = () -> PedroComponent.follower().getPose();
	public abstract Common.Alliance alliance();
	@Override
	public void onInit() {
		Common.alliance = alliance();
		addComponents(
				TelemetryComponent.INSTANCE,
				LoopTimeCompenent.INSTANCE,
				BulkReadComponent.INSTANCE,
				new PedroComponent(Constants::createFollower),
				new SubsystemComponent(IntakeTransfer.INSTANCE, RGBFlywheel.INSTANCE)
		);
		IntakeTransfer.INSTANCE.neutralKick.schedule();
	}
	public void onWaitForStart() {
		telemetry.addData("Variant", this.getClass().getSimpleName());

		LimeLightAprilTag ll = ((SensorFusion) follower().getPoseTracker().getLocalizer()).ll;

		Pose pose = follower().getPose();
		drawRobot(pose, "#4CAF50");
		telemetry.addData("Obelisk ID", formatObeliskID(ll.getObeliskID(pose)));
}
	public void onStop() {
		((SensorFusion) PedroComponent.follower().getPoseTracker().getLocalizer()).ll.close();
	}

	public abstract BasePaths getPaths(Follower follower);
	public void onStartButtonPressed() {
		RGBFlywheel.INSTANCE.setVelocity(1280).schedule();
		BasePaths paths = getPaths(PedroComponent.follower());

		new SequentialGroup(
				IntakeTransfer.INSTANCE.startIntake,
				new FollowPath(paths.Preload),
				preloadShootCommand(),
				new FollowPath(paths.GPP),
				new FollowPath(paths.GPPReturn),
				shootCommand(),
				new FollowPath(paths.PGP),
				new FollowPath(paths.PGPReturn),
				shootCommand(),
				new FollowPath(paths.PPG),
				new FollowPath(paths.PPGReturn),
				shootCommand(),
				new FollowPath(paths.LeaveZone)
		).schedule();
	}

	public Command shootCommand() {
		return new SequentialGroup(
				IntakeTransfer.INSTANCE.startFeeding,
				new ParallelDeadlineGroup(
						new Delay(5),
						new SequentialGroup(
								IntakeTransfer.INSTANCE.stopIntake,
								new Delay(0.2),
								IntakeTransfer.INSTANCE.startIntake,
								IntakeTransfer.INSTANCE.startChucking
						)
				),
				IntakeTransfer.INSTANCE.stopFeeding,
				IntakeTransfer.INSTANCE.stopChucking
		);
	}

	public Command preloadShootCommand() {
		return new SequentialGroup(
				IntakeTransfer.INSTANCE.startFeeding,
				IntakeTransfer.INSTANCE.startChucking,
				new Delay(3),
				IntakeTransfer.INSTANCE.stopChucking,
				IntakeTransfer.INSTANCE.stopFeeding
		);
	}
}