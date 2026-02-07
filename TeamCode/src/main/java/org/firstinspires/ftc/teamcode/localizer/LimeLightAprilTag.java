package org.firstinspires.ftc.teamcode.localizer;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.Drawing;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.stream.Collectors;

public class LimeLightAprilTag {
	Limelight3A limelight;
	PortForwarder forwarder;
	public LimeLightAprilTag(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "LimeLight3A");
		forwarder = new PortForwarder("172.29.0.1", 5800, 5801, 5802, 5803, 5804, 5805, 5806, 5807, 5808, 5809);
		forwarder.start();

		limelight.setPollRateHz(50);
		limelight.start();
	}

	public void updateRobotOrientation(double yaw) {
		limelight.updateRobotOrientation(Math.toDegrees(yaw));
	}

	public Optional<Pair<Pose, Long>> localizeRobotMT2() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			return Optional.of(Pair.create(flattenPose3DTo2d(result.getBotpose_MT2()), result.getControlHubTimeStampNanos()));
		}
		return Optional.empty();
	}

	public Optional<Pair<Pose, Long>> localizeRobotMT1() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			Pose3D pose = result.getBotpose();
			if (Common.telemetry != null) {
				Common.telemetry.addData("MT1 std dev", Arrays.toString(result.getStddevMt1()));
//				for (double x : result.getStddevMt1()) {
//					if (x > 20) {
//						return Optional.empty();
//					}
//				}
			}
			return Optional.of(Pair.create(flattenPose3DTo2d(pose), result.getControlHubTimeStampNanos()));
		}
		return Optional.empty();
	}

	public static Pose flattenPose3DTo2d(Pose3D pose3D) {
		Position p = pose3D.getPosition();
		DistanceUnit pUnit = p.unit;
		double x = p.x;
		double y = p.y;

		// extract yaw heading (radians)
		double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);

		return PoseConverter.pose2DToPose(new Pose2D(pUnit, x, y, AngleUnit.RADIANS, heading), InvertedFTCCoordinates.INSTANCE);
	}

	private Pose applyTransform(Pose robot, Pose local) {
		double x = robot.getX() + local.getX() * Math.cos(robot.getHeading()) - local.getY() * Math.sin(robot.getHeading());
		double y = robot.getY() + local.getX() * Math.sin(robot.getHeading()) + local.getY() * Math.cos(robot.getHeading());
		double h = robot.getHeading() + local.getHeading();
		return new Pose(x, y, h, robot.getCoordinateSystem());
	}

	public OptionalInt getObeliskID(Pose robotPose) {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			List<LLResultTypes.FiducialResult> fiducials = result
					.getFiducialResults()
					.stream()
					.filter(x ->
							x.getFiducialId() == 21 || x.getFiducialId() == 22 || x.getFiducialId() == 23
					) // filter for obelisk tags
					.collect(Collectors.toList());
			if (fiducials.isEmpty()) return OptionalInt.empty();

			if (fiducials.size() == 1) {
				return OptionalInt.of(fiducials.get(0).getFiducialId());
			} else {
				// return the apriltag that is facing the field
				for (LLResultTypes.FiducialResult fiducial : fiducials) {
					Pose aprilTagInRobotSpace = flattenPose3DTo2d(fiducial.getTargetPoseRobotSpace());
					Pose aprilTagInFieldSpace = applyTransform(robotPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE), aprilTagInRobotSpace); // apply transformation

					TelemetryPacket packet = new TelemetryPacket();
					packet.fieldOverlay().setStroke("#000000");
					Drawing.drawRobot(packet.fieldOverlay(), aprilTagInFieldSpace);
					FtcDashboard.getInstance().sendTelemetryPacket(packet);
					double heading = aprilTagInFieldSpace.getHeading(); // [-pi, pi]

					if (Math.abs(heading) < Math.toRadians(20)) {
						return OptionalInt.of(fiducial.getFiducialId());
					}
				}
			}
		}
		return OptionalInt.empty();
	}
	public void close() {
		forwarder.stop();
	}
}
