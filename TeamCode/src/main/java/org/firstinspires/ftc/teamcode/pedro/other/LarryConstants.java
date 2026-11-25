package org.firstinspires.ftc.teamcode.pedro.other;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LarryConstants {
    private static final double STARTER_MAX_WHEEL_POWER = 0.30;

    public static final MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftRear");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightRear");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

//    public static final PinpointConfig pinpointConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.xPodOffset.set(143.95);
//        c.yPodOffset.set(-90.1);
//        c.offsetUnits.set(DistanceUnit.MM);
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//    });

    // Starter estimates based on Pedro's example. Replace with LarryBoy's measurements when available.
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.proportional(0.10));
        c.strafeTranslational.set(Controller.proportional(0.10));
        c.headingFeedback.set(Controller.proportional(1.0));

        c.coast.set(Controller.proportionalFeedforward(0.011));
        c.brake.set(Controller.proportionalFeedforward(0.009));
        c.linearBrakeCoefficients.set(Matrix.diag(0.106, 0.087));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00147, 0.00138));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056, 0.0064));

        c.maxAchievableForwardVelocity.set(73.0);
        c.maxAchievableStrafeVelocity.set(52.0);
        c.naturalForwardDeceleration.set(85.0);
        c.naturalStrafeDeceleration.set(105.0);
        c.maxVelocityConstraint.set(18.0);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.571080846110667);
        c.yPodOffset.set(-2.275393778883566);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static Follower create(HardwareMap h) {
        Mecanum drivetrain = new Mecanum(h, mecanumConfig) {
            @Override
            public void applyDrive(DrivePowers powers) {
                // Bound the mixed wheel powers, including heading and lateral corrections.
                double combinedPower = Math.abs(powers.forward()) + Math.abs(powers.strafe())
                        + Math.abs(powers.turn());
                if (!Double.isFinite(combinedPower)) {
                    super.applyDrive(DrivePowers.zero());
                    return;
                }
                double scale = STARTER_MAX_WHEEL_POWER / Math.max(STARTER_MAX_WHEEL_POWER, combinedPower);
                super.applyDrive(new DrivePowers(powers.forward() * scale,
                        powers.strafe() * scale, powers.turn() * scale));
            }
        };
        PinpointLocalizer localizer = new PinpointLocalizer(h, localizerConfig);
        Foresight algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}
