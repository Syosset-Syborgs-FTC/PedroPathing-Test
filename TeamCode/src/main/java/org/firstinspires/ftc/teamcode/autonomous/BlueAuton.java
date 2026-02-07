package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;

@Autonomous
public class BlueAuton extends SyborgsAutonBase {
	@Override
	public Common.Alliance alliance() {
		return Common.Alliance.Blue;
	}

	public BasePaths getPaths(Follower follower) {
		return new Paths(follower);
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
//									new Pose(56.000, 8.000),
									new Pose(60.307, 79.971),
									new Pose(52.859, 89.897)
							)
					).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))

					.build();

			GPP = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(52.859, 89.897),
									new Pose(53.743, 66.955),
									new Pose(50.618, 47.398),
									new Pose(52.015, 29.681),
									new Pose(9.425, 33.065)
							)
					).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

					.build();

			GPPReturn = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(9.425, 33.065),
									new Pose(56.243, 35.582),
									new Pose(53.100, 89.728)
							)
					).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

					.build();

			PGP = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(53.100, 89.728),
									new Pose(59.736, 58.157),
									new Pose(41.270, 56.495),
									new Pose(11.600, 57.784)
							)
					).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

					.build();

			PGPReturn = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(11.600, 57.784),
									new Pose(47.514, 61.710),
									new Pose(53.071, 89.773)
							)
					).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

					.build();

			PPG = follower.pathBuilder().addPath(
							new BezierCurve(
									currentPose,
//									new Pose(53.071, 89.773),
									new Pose(46.968, 80.977),
									new Pose(18.563, 82.944)
							)
					).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

					.build();

			PPGReturn = follower.pathBuilder().addPath(
							new BezierLine(
									currentPose,
//									new Pose(18.563, 82.944),

									new Pose(53.063, 89.408)
							)
					).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

					.build();

			LeaveZone = follower.pathBuilder().addPath(
							new BezierLine(
									currentPose,
//									new Pose(53.063, 89.408),

									new Pose(20.968, 70.039)
							)
					).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

					.build();
		}
	}
}
