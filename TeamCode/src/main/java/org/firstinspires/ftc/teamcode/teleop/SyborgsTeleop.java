package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Common.formatObeliskID;
import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.sendPacket;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.gamepad1;
import static dev.nextftc.ftc.Gamepads.gamepad2;

import android.util.Pair;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.components.LoopTimeCompenent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.control.HeadingController;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.AutoSort;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransfer;
import org.firstinspires.ftc.teamcode.subsystems.RGBFlywheel;

import java.util.Optional;
import java.util.OptionalInt;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp
public class SyborgsTeleop extends NextFTCOpMode {
	public static final double NORMAL_VELOCITY = 1350;
	public static final double FAR_VELOCITY = 2000;
	public static volatile double targetVelocity = NORMAL_VELOCITY;
	public boolean flywheelEnabled = false;
	public static volatile double anglerPosition = 0.1;
	HeadingController headingController = new HeadingController();
	private boolean autoAlign = false;
	AutoSort autoSort;

	double headingOffset = Math.toRadians(0);
	boolean slowDrive = false;
	boolean autoPark = false;

	public SyborgsTeleop() {
		addComponents(
				TelemetryComponent.INSTANCE,
				LoopTimeCompenent.INSTANCE,
				BulkReadComponent.INSTANCE,
				BindingsComponent.INSTANCE,
				new PedroComponent(Constants::createFollower),
				CommandManager.INSTANCE,
				new SubsystemComponent(RGBFlywheel.INSTANCE, IntakeTransfer.INSTANCE)
		);
	}

	@Override
	public void onInit() {
		targetVelocity = NORMAL_VELOCITY;

		autoSort = new AutoSort(hardwareMap, telemetry);
		follower().startTeleopDrive(true);


		handleInput();
	}

	@Override
	public void onUpdate() {
		double cycleStart = getRuntime();
		OptionalInt currentObeliskID = ((SensorFusion) follower().getPoseTracker().getLocalizer()).getObeliskID();
		autoSort.update();
		driveRobot();
		new ParallelGroup(
				RGBFlywheel.INSTANCE.setAnglerPosition(() -> anglerPosition),
				RGBFlywheel.INSTANCE.setVelocity(() -> flywheelEnabled? targetVelocity : 0)
		).schedule();

	}

	@Override
	public void onStop() {
		((SensorFusion) follower().getPoseTracker().getLocalizer()).ll.close();
	}

	private void handleInput() {
		handleManualAdjust();
		handleShooterInput();
		handleDriveConfig();

	}

	private void handleDriveConfig() {
		gamepad1().y()
				.whenBecomesTrue(() -> autoAlign = !autoAlign)
				.toggleOnBecomesTrue()
				.whenBecomesFalse(() -> headingController.reset());

		gamepad1().a()
				.whenBecomesTrue(() -> slowDrive = !slowDrive);
		gamepad1().x()
				.whenBecomesTrue(() ->
						headingOffset = ((SensorFusion) follower().getPoseTracker().getLocalizer()).getRawPinpointHeading()
				);

		Pose pose = follower().getPose();
		Pose autoParkPose = Common.alliance == Common.Alliance.Red ? new Pose(38.641, 33.355, Math.toRadians(90)) : new Pose(101.359, 33.355, Math.toRadians(90));
		gamepad1().b()
				.toggleOnBecomesTrue()
				.whenBecomesTrue(() -> {
					autoPark = true;
					Path path = new Path(new BezierLine(pose, autoParkPose));
					path.setLinearHeadingInterpolation(pose.getHeading(), autoParkPose.getHeading());
					follower().followPath(path);
				})
				.whenBecomesFalse(() -> {
					autoPark = false;
					follower().startTeleopDrive(true);
				});
	}

	private void handleShooterInput() {
		gamepad1().leftTrigger().atLeast(0.5)
				.whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);

		gamepad1().rightTrigger().atLeast(0.5)
				.whenFalse(IntakeTransfer.INSTANCE.stopChucking)
				.toggleOnBecomesTrue()
				.whenBecomesTrue(new ParallelGroup(
						IntakeTransfer.INSTANCE.startFeeding,
						IntakeTransfer.INSTANCE.startChucking
				)).whenBecomesFalse(IntakeTransfer.INSTANCE.stopFeeding);

		gamepad1().rightBumper()
				.whenBecomesTrue(IntakeTransfer.INSTANCE.toggleIntake);
		gamepad1().leftBumper()
				.whenBecomesTrue(IntakeTransfer.INSTANCE.toggleOuttake);

		gamepad1().dpadUp()
				.whenTrue(() -> targetVelocity = FAR_VELOCITY);
		gamepad1().dpadDown()
				.whenTrue(() -> targetVelocity = NORMAL_VELOCITY);
		gamepad1().dpadRight()
				.whenTrue(IntakeTransfer.INSTANCE.startKicking)
				.whenFalse(IntakeTransfer.INSTANCE.stopKicking);
	}

	private static void handleManualAdjust() {
		gamepad2().y()
				.whenBecomesTrue(() -> adjustAnglerPosition(0.05));
		gamepad2().a()
				.whenBecomesTrue(() -> adjustAnglerPosition(-0.05));

		gamepad2().dpadUp()
				.whenBecomesTrue(() -> adjustTargetVelocity(25));
		gamepad2().dpadDown()
				.whenBecomesTrue(() -> adjustTargetVelocity(-25));
	}

	public synchronized static void adjustTargetVelocity(double delta) {
		targetVelocity += delta;
	}

	public synchronized static void adjustAnglerPosition(double delta) {
		anglerPosition += delta;
	}

	private void driveRobot() {
		double forward = -gamepad1.left_stick_y;
		double strafe = -gamepad1.left_stick_x;
		double rotate = -gamepad1.right_stick_x;

		if (slowDrive) {
			forward *= 0.4;
			strafe *= 0.4;
			rotate *= 0.4;
		}
		telemetry.addData("Auto Align", autoAlign);
		follower().update();

		Pose pose = follower().getPose();
		double alignX = -141;
		double alignY = Common.alliance == Common.Alliance.Blue ? 6 : 138;
		double turnPower = headingController.getTurnPower(pose, alignX, alignY);
		telemetry.addData("Turn Power", turnPower);

		Vector input = new Vector();
		input.setOrthogonalComponents(forward, strafe);

		input.rotateVector(-((SensorFusion) follower().getPoseTracker().getLocalizer()).getRawPinpointHeading());
		follower().setTeleOpDrive(input.getXComponent(), input.getYComponent(), autoAlign? turnPower : rotate, true, headingOffset);
		Drawing.panelsField.setStyle(new Style("#FFFFFF", "#FFFFFF", 1));
		Drawing.panelsField.moveCursor(0, 0);
		Drawing.panelsField.line(input.getXComponent()*10, input.getYComponent()*10);
		if (autoPark && !follower().isBusy()) {
			follower().startTeleopDrive(true);
		}
		telemetry.addData("Auto Park", autoPark);
		telemetry.addData("Heading Offset", headingOffset);
		sendPoseToDash(pose);
	}

	@Override
	public void onWaitForStart() {
		follower().update();

		drawRobot(follower().getPose(),"#4CAF50");
		sendPacket();

		telemetry.addData("Alliance (press right bumper to change): ", Common.alliance.toString());
		gamepad1().rightBumper()
				.whenBecomesTrue(() -> Common.alliance = Common.alliance.getOpposite());
	}


	private void sendPoseToDash(Pose pose) {
		LimeLightAprilTag ll = ((SensorFusion) follower().getPoseTracker().getLocalizer()).ll;
		Optional<Pair<Pose, Long>> mt1Pose = ll.localizeRobotMT1();
		Optional<Pair<Pose, Long>> mt2Pose = ll.localizeRobotMT2();

		telemetry.addData("Obelisk ID", formatObeliskID(ll.getObeliskID(pose)));

		mt1Pose.ifPresent(p -> drawRobot(p.first, "#4CAF50"));
		mt2Pose.ifPresent(p -> drawRobot(p.first, "#FF5722"));
		drawRobot(pose, "#3F51B5");
		sendPacket();

		telemetry.addData("Pose (x, y, headingDegrees)", "%.2f, %.2f, %.2f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
	}
}