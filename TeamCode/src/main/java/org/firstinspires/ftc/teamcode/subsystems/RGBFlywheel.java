package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.PIDFController;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class RGBFlywheel implements Subsystem {
	public static final RGBFlywheel INSTANCE = new RGBFlywheel();
	MotorEx flywheel;
	ServoEx rgbLight;

	public volatile static double kP = 0.002;
	public volatile static double kI = 0.00052;
	public volatile static double kD = 0.0001;
	public volatile static double kF = 0.0052;
	private PIDFController flywheelController;
	VoltageSensor voltage;
	public double targetVelocity = 0.0;
	public boolean autoAlignIndicator;
	public Command setVelocity(double velocity) {
		return new InstantCommand(() -> this.targetVelocity = velocity);
	}
	public Command setVelocity(DoubleSupplier velocity) {
		return new InstantCommand(() -> this.targetVelocity = velocity.getAsDouble());
	}
	public Command enableAutoAlign = new InstantCommand(() -> this.autoAlignIndicator = true);
	public Command disableAutoAlign = new InstantCommand(() -> this.autoAlignIndicator = false);
	ServoEx angler;
	public Command setAnglerPosition(double position) {
		return new SetPosition(angler, position);
	}
	public Command setAnglerPosition(DoubleSupplier position) {
		return new InstantCommand(() -> angler.setPosition(position.getAsDouble()));
	}

	public double getAnglerPosition() {
		return angler.getPosition();
	}
	@Override
	public void initialize() {
		voltage = ActiveOpMode.hardwareMap().voltageSensor.get("Expansion Hub 2");
		flywheelController = new PIDFController(kP, kI, kD);
		flywheel = new MotorEx("st");
		rgbLight = new ServoEx("rgb");
		angler = new ServoEx("angle");
	}
	@Override
	public void periodic() {
		double currentVelocity = updateFlywheel();
		updateRGB(currentVelocity);
		ActiveOpMode.telemetry().addData("Angler Position", angler.getPosition());
	}

	private void updateRGB(double currentVelocity) {
		if (targetVelocity == 0) {
			rgbLight.setPosition(0.28); // red
		} else if (Math.abs(targetVelocity - currentVelocity) > 100) {
			rgbLight.setPosition(0.28);
		} else if (currentVelocity < targetVelocity - 20) {
			rgbLight.setPosition(0.611); // blue
		} else if (currentVelocity > targetVelocity + 20) {
			rgbLight.setPosition(0.333); // orange
		} else {
			if (autoAlignIndicator) {
				rgbLight.setPosition(0.999); // white
			} else {
				rgbLight.setPosition(0.5); // green
			}
		}
	}

	private double updateFlywheel() {
		double reading = voltage.getVoltage();
		flywheelController.setConstants(kP, kI, kD);
		flywheelController.setTarget(targetVelocity);
		double currentVelocity = flywheel.getVelocity();

		double power = flywheelController.update(currentVelocity, targetVelocity * kF / reading);
		Telemetry telemetry = ActiveOpMode.telemetry();
		if (flywheel.getMotor().isOverCurrent()) {
			telemetry.addLine("Flywheel is over current!");
		}
		telemetry.addData("Flywheel", currentVelocity);
		telemetry.addData("Flywheel Target", targetVelocity);
		telemetry.addData("Flywheel Current", flywheel.getMotor().getCurrent(CurrentUnit.AMPS));
		telemetry.addData("Flywheel Power", power);

		flywheel.setPower(power);
		return currentVelocity;
	}
}
