package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz Red Near", group = "BioBuzz")
public final class BioBuzzAutoRedNear extends BioBuzzAuto {
    private static final Pose START = new Pose(56, 135, Math.toRadians(-90));
    private static final Pose LAUNCH = new Pose(56, 129, Math.toRadians(-90));
    private static final Pose PARK = new Pose(10, 108, Math.toRadians(-90));
    private static final Pose PARK_WAYPOINT = new Pose(26, 129, Math.toRadians(-90));

    public BioBuzzAutoRedNear() {
        super(Alliance.RED, StartPosition.NEAR, START, LAUNCH, PARK);
    }

    @Override
    protected Path buildApproachPath() {
        return Paths.line(START, LAUNCH).linear(START, LAUNCH);
    }

    @Override
    protected Path buildParkingPath() {
        return Paths.path(
                Paths.line(LAUNCH, PARK_WAYPOINT).constant(LAUNCH.heading()),
                Paths.line(PARK_WAYPOINT, PARK).constant(LAUNCH.heading()));
    }
}
