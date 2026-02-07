package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
@Autonomous
public class RedAuton extends SyborgsAutonBase {
	@Override
	public Common.Alliance alliance() {
		return Common.Alliance.Red;
	}
	public BasePaths getPaths(Follower follower) {
		return new RedAuton.Paths(follower);
	}

	public static class Paths extends BasePaths {
		public PathChain Preload;
		public PathChain GPP;
		public PathChain GPPReturn;
		public PathChain PGP;
		public PathChain PGPReturn;
		public PathChain PPG;
		public PathChain PPGReturn;
		public PathChain LeaveZone;

		public Paths(Follower follower) {
			Preload = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(88.000, 8.000),
									new Pose(83.693, 79.971),
									new Pose(91.141, 89.897)
							)
					).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))

					.build();

			GPP = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(91.141, 89.897),
									new Pose(90.257, 66.955),
									new Pose(93.382, 47.398),
									new Pose(91.985, 29.681),
									new Pose(134.575, 33.065)
							)
					).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

					.build();

			GPPReturn = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(134.575, 33.065),
									new Pose(87.757, 35.582),
									new Pose(90.900, 89.728)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

					.build();

			PGP = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(90.900, 89.728),
									new Pose(84.264, 58.157),
									new Pose(102.730, 56.495),
									new Pose(132.400, 57.784)
							)
					).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

					.build();

			PGPReturn = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(132.400, 57.784),
									new Pose(96.486, 61.710),
									new Pose(90.929, 89.773)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

					.build();

			PPG = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(90.929, 89.773),
									new Pose(97.032, 80.977),
									new Pose(125.437, 82.944)
							)
					).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

					.build();

			PPGReturn = follower.pathBuilder().addPath(
							new BezierLine(
									currentPose,
//									new Pose(125.437, 82.944),

									new Pose(90.937, 89.408)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

					.build();

			LeaveZone = follower.pathBuilder().addPath(
							new BezierLine(
									currentPose,
//									new Pose(90.937, 89.408),

									new Pose(123.032, 70.039)
							)
					).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

					.build();
		}
	}

}
