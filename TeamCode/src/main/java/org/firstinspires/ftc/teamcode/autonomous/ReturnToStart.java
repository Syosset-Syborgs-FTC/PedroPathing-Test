package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.RGBFlywheel;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public abstract class ReturnToStart extends NextFTCOpMode {
	@Override
	public void onInit() {
		addComponents(
				BulkReadComponent.INSTANCE,
				new PedroComponent(Constants::createFollower),
				new SubsystemComponent(IntakeTransfer.INSTANCE, RGBFlywheel.INSTANCE)
		);
	}

	public abstract BasePaths getPaths(Follower follower);

	public void onStartButtonPressed() {
		BasePaths paths = getPaths(PedroComponent.follower());
		double x =  81.507;
		if (Common.alliance == Common.Alliance.Blue) {
			x = 144-x;
		}
		Path path = new Path(
				new BezierLine(
						PedroComponent.follower()::getPose,
						new Pose(x, 9.258)
				)
		);
		path.setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), Math.toRadians(90));
		new FollowPath(path).schedule();
	}

}

