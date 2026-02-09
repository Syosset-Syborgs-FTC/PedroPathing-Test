package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localizer.SensorFusion;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
	public static PinpointConstants localizerConstants = new PinpointConstants()
			.forwardPodY(-3.5058448819)
			.strafePodX(-7.14888700787)
			.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
			.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName("FL")
			.rightRearMotorName("BR")
			.rightFrontMotorName("FR")
			.leftRearMotorName("BL")
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
		SensorFusion fusion = new SensorFusion(hardwareMap, localizerConstants);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
				.setLocalizer(fusion)
				.mecanumDrivetrain(driveConstants)
                .build();
    }
}
