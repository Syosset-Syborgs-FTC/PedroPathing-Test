package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.Common;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class PoseFilter {

	private final NavigableMap<Long, Pose> odomHistory = new TreeMap<>();

	// tracking offset between raw pinpoint pose and world pose
	private Pose offset = new Pose(0, 0, 0);

	// kalman filter covariance
	private double covX = 0;
	private double covY = 0;
	private double covH = 0;


	// q: process noise covariance how fast odometry drifts
	private final double Q_POS = 0.002;
	private final double Q_HEAD = 0.001;

	// r: measurement noisy covariance
	private final double R_POS = 0.5; // inch^2 variance
	private final double R_HEAD = 0.05; // rad^2 variance

	// outliers
	private final double GATE_DISTANCE = 6.0; // inch
	private final double GATE_HEADING = Math.toRadians(15.0);
	private final int OUTLIER_COUNT = 10;
	private int suspiciousReadingCounter = 0;

	private final long HISTORY_RETENTION_MS = 1000;
	private boolean isVisionInitialized = false;


	public Pose getPose(Pose rawPose) {
		return Common.applyTransform(offset, rawPose);
	}

	public void updateOdometry(Pose rawPinpointPose, long timestampNs) {
		// store history
		odomHistory.put(timestampNs, rawPinpointPose);
		long cleanupThreshold = timestampNs - (HISTORY_RETENTION_MS * 1_000_000);
		odomHistory.headMap(cleanupThreshold).clear();

		covX += Q_POS;
		covY += Q_POS;
		covH += Q_HEAD;
	}
	public void setPose(Pose rawPinpointPose, Pose desiredWorldPose) {
		this.offset = Common.applyTransform(desiredWorldPose, Common.inverse(rawPinpointPose));

		this.covX = R_POS;
		this.covY = R_POS;
		this.covH = R_HEAD;
	}
	public void updateVision(Pose visionPose, long timestampNs) {
		Pose rawPoseAtTime = getInterpolatedHistory(timestampNs);

		// assume 0 if history is missing
		if (rawPoseAtTime == null) {
			rawPoseAtTime = new Pose(0, 0, 0);
		}

		Pose measuredOffset = Common.applyTransform(visionPose, Common.inverse(rawPoseAtTime));

		if (!isVisionInitialized) {
			initializeFilter(measuredOffset);
			return;
		}

		Pose estimatedPoseAtTime = Common.applyTransform(offset, rawPoseAtTime);
		double distError = estimatedPoseAtTime.distanceFrom(visionPose);
		double headingError = Math.abs(getSmallestAngleDifference(estimatedPoseAtTime.getHeading(), visionPose.getHeading()));

		boolean isSuspicious = (distError > GATE_DISTANCE) || (headingError > GATE_HEADING);

		if (isSuspicious) {
			suspiciousReadingCounter++;
			if (suspiciousReadingCounter > OUTLIER_COUNT) {
				// reset filter if too many bad readings in a row
				initializeFilter(measuredOffset);
			}
		} else {
			suspiciousReadingCounter = 0;
			performKalmanCorrection(measuredOffset); // update
		}
	}

	private void performKalmanCorrection(Pose measuredOffset) {
		// kalman gains k = p / (p + r)
		double K_x = covX / (covX + R_POS);
		double K_y = covY / (covY + R_POS);
		double K_h = covH / (covH + R_HEAD);

		// update state (offset = offset + k * (measurement - offset))
		double newX = offset.getX() + K_x * (measuredOffset.getX() - offset.getX());
		double newY = offset.getY() + K_y * (measuredOffset.getY() - offset.getY());

		// handle heading wrap for error
		double headingDelta = getSmallestAngleDifference(measuredOffset.getHeading(), offset.getHeading());
		double newH = offset.getHeading() + K_h * headingDelta;

		this.offset = new Pose(newX, newY, newH);

		// update covariance (p = (1 - k) * p)
		covX = (1.0 - K_x) * covX;
		covY = (1.0 - K_y) * covY;
		covH = (1.0 - K_h) * covH;
	}

	private void initializeFilter(Pose startingOffset) {
		this.offset = startingOffset;
		this.isVisionInitialized = true;
		this.suspiciousReadingCounter = 0;

		// change covariance to uncertain
		this.covX = R_POS;
		this.covY = R_POS;
		this.covH = R_HEAD;
	}

	private Pose getInterpolatedHistory(long timestampNs) {
		Map.Entry<Long, Pose> floor = odomHistory.floorEntry(timestampNs);
		Map.Entry<Long, Pose> ceiling = odomHistory.ceilingEntry(timestampNs);

		if (floor == null && ceiling == null) return null;
		if (floor == null) return ceiling.getValue();
		if (ceiling == null) return floor.getValue();

		long dFloor = Math.abs(timestampNs - floor.getKey());
		long dCeil = Math.abs(timestampNs - ceiling.getKey());

		if (dFloor + dCeil == 0) return floor.getValue();

		double alpha = (double) dFloor / (dFloor + dCeil);

		// linear interpolation
		double newX = floor.getValue().getX() * (1 - alpha) + ceiling.getValue().getX() * alpha;
		double newY = floor.getValue().getY() * (1 - alpha) + ceiling.getValue().getY() * alpha;

		// interpolation of heading (handling wrap)
		double h1 = floor.getValue().getHeading();
		double h2 = ceiling.getValue().getHeading();
		double hDiff = getSmallestAngleDifference(h2, h1);
		double newH = h1 + hDiff * alpha;

		return new Pose(newX, newY, newH);
	}

	private double getSmallestAngleDifference(double target, double source) {
		return MathFunctions.normalizeAngleSigned(target - source);
	}

	public void reset() {
		offset = new Pose(0,0,0);
		suspiciousReadingCounter = 0;
		odomHistory.clear();
		isVisionInitialized = false;
		covX = 0; covY = 0; covH = 0;
	}
}
