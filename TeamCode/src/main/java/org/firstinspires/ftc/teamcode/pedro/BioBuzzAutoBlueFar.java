package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz Blue Far", group = "BioBuzz")
public final class BioBuzzAutoBlueFar extends BioBuzzAuto {
    private static final Pose START = new Pose(88, 136, Math.toRadians(270));
    private static final Pose LAUNCH = new Pose(87.6641, 122.276, Math.toRadians(360));
    private static final Pose PARK = new Pose(139.0328, 45.9779, Math.toRadians(123.9509));

    public BioBuzzAutoBlueFar() {
        super(Alliance.BLUE, StartPosition.FAR, START, LAUNCH, PARK);
    }

    @Override
    protected Path buildApproachPath() {
        return Paths.line(START, LAUNCH).linear(START, LAUNCH);
    }

    @Override
    protected Path buildParkingPath() {
        return Paths.line(LAUNCH, PARK).reverseTangent();
    }
}
