package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
	private static final double EPS = 1e-9;

	private double kP, kI, kD;
	private double target = 0.0;
	private boolean enabled = true;

	private double integralSum = 0.0;
	private double lastError = 0.0;
	private double lastMeasurement = 0.0;

	private double filteredDerivative = 0.0;

	private double lastTime = 0.0;
	private boolean initialized = false;

	private double outputMin = -1.0;
	private double outputMax = 1.0;
	private double maxIntegralContribution = 1.0;

	private double derivativeTau = 0.02;
	// continuous heading mode
	private boolean continuous = false;
	private double inputMin = 0.0;
	private double inputMax = 0.0;

	public PIDFController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public static double time() {
		return System.nanoTime() / 1e9;
	}

	public void setContinuous(boolean continuous, double minInput, double maxInput) {
		this.continuous = continuous;
		this.inputMin = minInput;
		this.inputMax = maxInput;
		if (continuous) {
			if (maxInput <= minInput) throw new IllegalArgumentException("maxInput must be > minInput");
		}
	}

	public void setTarget(double target, boolean resetIntegrator) {
		this.target = target;
		if (resetIntegrator) {
			reset();
		}
	}

	public void setTarget(double target) {
		setTarget(target, false);
	}

	public void setEnabled(boolean enabled) {
		if (enabled && !this.enabled) {
			reset();
		}
		this.enabled = enabled;
	}

	public boolean getEnabled() {
		return enabled;
	}

	public double getTarget() {
		return target;
	}

	public void setOutputLimits(double min, double max) {
		if (max <= min) throw new IllegalArgumentException("max must be > min");
		this.outputMin = min;
		this.outputMax = max;
	}

	public void setMaxIntegralContribution(double maxContribution) {
		this.maxIntegralContribution = Math.abs(maxContribution);
	}

	public void setDerivativeTau(double tauSeconds) {
		this.derivativeTau = Math.max(tauSeconds, 1e-9);
	}

	public void reset() {
		initialized = false;
	}


	private double wrapError(double error) {
		if (!continuous) return error;
		double range = inputMax - inputMin;
		if (range <= 0) return error;


		return Math.IEEEremainder(error, range);
	}


	public double update(double measurement, double feedForward) {
		if (!enabled) return 0.0;

		double now = time();
		double rawError = target - measurement;
		double error = wrapError(rawError);

		if (!initialized) {
			lastTime = now;
			lastError = error;
			lastMeasurement = measurement;
			filteredDerivative = 0.0;
			integralSum = 0.0;
			initialized = true;
			double p = kP * lastError;
			return Range.clip(p + feedForward, outputMin, outputMax);
		}

		double dt = now - lastTime;
		lastTime = now;
		if (dt < 1e-9) dt = 1e-9;

		double pTerm = kP * error;

		double measurementDiff = lastMeasurement - measurement;
		measurementDiff = wrapError(measurementDiff);

		double rawDerivative = measurementDiff / dt;
		double alpha = dt / (derivativeTau + dt);
		filteredDerivative = alpha * rawDerivative + (1.0 - alpha) * filteredDerivative;
		double dTerm = kD * filteredDerivative;

		double iTerm = 0.0;

		if (Math.abs(kI) > 1e-9) {
			double integralProposed = integralSum + 0.5 * (error + lastError) * dt;

			double maxIntegral = maxIntegralContribution / Math.abs(kI);
			integralProposed = Range.clip(integralProposed, -maxIntegral, maxIntegral);

			double iTermProposed = kI * integralProposed;

			double unclipped = pTerm + iTermProposed + dTerm + feedForward;
			double clipped = Range.clip(unclipped, outputMin, outputMax);

			boolean wouldSaturate = Math.abs(unclipped - clipped) > EPS;

			if (!wouldSaturate) {
				integralSum = integralProposed;
			} else {
				double currentITerm = kI * integralSum;
				boolean reducesSaturation =
						(unclipped > outputMax && iTermProposed < currentITerm) ||
								(unclipped < outputMin && iTermProposed > currentITerm);

				if (reducesSaturation) {
					integralSum = integralProposed;
				}
			}
			iTerm = kI * integralSum;
		} else {
			integralSum = 0.0;
		}

		lastError = error;
		lastMeasurement = measurement;

		double output = pTerm + iTerm + dTerm + feedForward;
		return Range.clip(output, outputMin, outputMax);
	}

	public void setConstants(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
}
