package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class IntakeTransfer  implements Subsystem {
	public static final IntakeTransfer INSTANCE = new IntakeTransfer();
	boolean feeding = false;
	MotorEx intake = new MotorEx("in");
	int intakeState = 0;
	double intakeStartTime = 0;
	public Command startIntake = new InstantCommand(() -> {
		intakeStartTime = ActiveOpMode.getRuntime();
		intakeState = 1;
	});
	public Command startOuttake = new InstantCommand(() -> {
		intakeState = -1;
	});
	public Command stopIntake = new InstantCommand(() -> {
		intakeState = 0;
	});
	public Command toggleIntake = new InstantCommand(() -> {
		if (intakeState == 1) {
			stopIntake.run();
		}
		else {
			startIntake.run();
		}
	});
	public Command toggleOuttake = new InstantCommand(() -> {
		if (intakeState == -1) {
			stopIntake.run();
		}
		else {
			startOuttake.run();
		}
	});
	CRServoEx transfer = new CRServoEx("tn");
	CRServoEx cycle = new CRServoEx("ts");

	ServoEx chuck = new ServoEx("c");
	ServoEx kicker = new ServoEx("k");
	public Command startFeeding = new InstantCommand(() -> {
		feeding = true;
	});
	public Command stopFeeding = new InstantCommand(() -> {
		feeding = false;
	});

	public Command startChucking = new SetPosition(chuck, 1);
	public Command stopChucking = new SetPosition(chuck, 0.2);
	public Command startKicking = new SetPosition(kicker, 0.3);
	public Command stopKicking = new SetPosition(kicker, 0.1);
	public Command neutralKick = new SetPosition(kicker, 0);
	public Command toggleFeeding = new InstantCommand(() -> {
		if (!feeding) {
			startChucking.schedule();
		}
		feeding = !feeding;
	});
	public void initialize() {
		new SequentialGroup(
				stopIntake,
				stopFeeding,
				stopChucking,
				stopKicking
		).schedule();
	}

	@Override
	public void periodic() {
		if (intakeState == 0) {
			intake.setPower(0);
			if (feeding) {
				transfer.setPower(1);
				cycle.setPower(1);
			} else {
				transfer.setPower(0);
				cycle.setPower(0);
			}
		}
		if (intakeState == 1) {
			if ((ActiveOpMode.getRuntime() - intakeStartTime) < 0.1) {
				intake.setPower(-1);
			} else {
				intake.setPower(1);
			}
			cycle.setPower(1);
			if (feeding) {
				transfer.setPower(1);
			} else {
				transfer.setPower(0);
			}
		}
		if (intakeState == -1) {
			intake.setPower(-1);
			cycle.setPower(-1);
			transfer.setPower(-1);
		}
	}
}
