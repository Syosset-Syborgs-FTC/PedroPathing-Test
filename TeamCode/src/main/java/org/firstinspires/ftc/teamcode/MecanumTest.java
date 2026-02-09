package org.firstinspires.ftc.teamcode;

import dev.nextftc.bindings.Bindings;
import static dev.nextftc.ftc.Gamepads.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
@TeleOp
public class MecanumTest extends NextFTCOpMode {
	MotorEx fl = new MotorEx(Constants.driveConstants.leftFrontMotorName);
	MotorEx fr = new MotorEx(Constants.driveConstants.rightFrontMotorName);
	MotorEx bl = new MotorEx(Constants.driveConstants.leftRearMotorName);
	MotorEx br = new MotorEx(Constants.driveConstants.rightRearMotorName);
	@Override
	public void onInit() {
		addComponents(BindingsComponent.INSTANCE);
		fl.getMotor().setDirection(Constants.driveConstants.leftFrontMotorDirection);
		fr.getMotor().setDirection(Constants.driveConstants.rightFrontMotorDirection);
		bl.getMotor().setDirection(Constants.driveConstants.leftRearMotorDirection);
		br.getMotor().setDirection(Constants.driveConstants.rightRearMotorDirection);
		gamepad1().a().whenTrue(new SetPower(fl, 1)).whenFalse(new SetPower(fl, 0));
		gamepad1().b().whenTrue(new SetPower(fr, 1)).whenFalse(new SetPower(fr, 0));
		gamepad1().x().whenTrue(new SetPower(bl, 1)).whenFalse(new SetPower(bl, 0));
		gamepad1().y().whenTrue(new SetPower(br, 1)).whenFalse(new SetPower(br, 0));
	}
}
