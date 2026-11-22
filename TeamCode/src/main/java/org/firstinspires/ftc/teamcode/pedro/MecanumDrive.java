package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class MecanumDrive {
    private static final double SLOW_MOVE_SCALE = 0.5;
    private static final double SLOW_TURN_SCALE = 0.8;

    private final Mecanum drivetrain;

    public MecanumDrive(HardwareMap hardwareMap) {
        drivetrain = BiobuzzConstants.createDrivetrain(hardwareMap);
    }

    public void drive(double forward, double strafe, double turn, boolean slow) {
        double moveScale = slow ? SLOW_MOVE_SCALE : 1.0;
        double turnScale = slow ? SLOW_TURN_SCALE : 1.0;
        // Convert the TeleOp joystick signs to Pedro's strafe and turn conventions.
        drivetrain.applyDrive(new DrivePowers(forward * moveScale, -strafe * moveScale, -turn * turnScale));
    }
}
