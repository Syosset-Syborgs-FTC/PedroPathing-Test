package org.firstinspires.ftc.teamcode.localizer;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.PoseFilter;

import java.util.Objects;
import java.util.OptionalInt;

public class SensorFusion implements Localizer {
	Pose startPose;

	PoseFilter filter = new PoseFilter();
	PinpointLocalizer pinpointLocalizer;
	public LimeLightAprilTag ll;

	public SensorFusion(HardwareMap hardwareMap, PinpointConstants pinpointConstants) {
		pinpointLocalizer = new PinpointLocalizer(hardwareMap, pinpointConstants, new Pose(0, 0, 0));
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());
		ll = new LimeLightAprilTag(hardwareMap);
	}
	@Override
	public Pose getPose() {
		return filter.getPose(pinpointLocalizer.getPose());
	}

	@Override
	public Pose getVelocity() {
		return filter.getPose(pinpointLocalizer.getVelocity()); // TODO: check if this is correct
	}

	@Override
	public Vector getVelocityVector() {
		return getVelocity().getAsVector();
	}


	@Override
	public void setStartPose(Pose setStart) {
		if (!Objects.equals(startPose, new Pose()) && startPose != null) {
			Pose currentPose = getPose().rotate(-startPose.getHeading(), false).minus(startPose);
			setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
		} else {
			setPose(setStart);
		}

		this.startPose = setStart;
	}


	@Override
	public void setPose(Pose setPose) {
		filter.setPose(pinpointLocalizer.getPose(), setPose);
	}

	@Override
	public void update() {
		pinpointLocalizer.update();
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());
		ll
			.localizeRobotMT1()
			.ifPresent(pair -> {
				filter.updateVision(pair.first, pair.second);
			});
		ll.updateRobotOrientation(filter.getPose(pinpointLocalizer.getPose()).getHeading());
	}

	@Override
	public double getTotalHeading() {
		return pinpointLocalizer.getTotalHeading();
	}

	@Override
	public double getForwardMultiplier() {
		return pinpointLocalizer.getForwardMultiplier();
	}

	@Override
	public double getLateralMultiplier() {
		return pinpointLocalizer.getLateralMultiplier();
	}

	@Override
	public double getTurningMultiplier() {
		return pinpointLocalizer.getTurningMultiplier();
	}

	@Override
	public void resetIMU() throws InterruptedException {
		pinpointLocalizer.resetIMU();
	}

	@Override
	public double getIMUHeading() {
		return pinpointLocalizer.getIMUHeading();
	}

	@Override
	public boolean isNAN() {
		Pose pose = getPose();
		return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
	}
	public double getRawPinpointHeading() {
		return pinpointLocalizer.getPose().getHeading();
	}
	public OptionalInt getObeliskID() {
		return ll.getObeliskID(getPose());
	}
}
