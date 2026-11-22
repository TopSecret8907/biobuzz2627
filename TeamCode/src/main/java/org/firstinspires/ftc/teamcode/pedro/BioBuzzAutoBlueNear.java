package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz Blue Near", group = "BioBuzz")
public final class BioBuzzAutoBlueNear extends BioBuzzAuto {
    private static final Pose START = new Pose(88, 9, Math.toRadians(90));
    private static final Pose LAUNCH = new Pose(88, 15, Math.toRadians(90));
    private static final Pose PARK = new Pose(134, 36, Math.toRadians(90));
    private static final Pose PARK_WAYPOINT = new Pose(118, 15, Math.toRadians(90));

    public BioBuzzAutoBlueNear() {
        super(Alliance.BLUE, StartPosition.NEAR, START, LAUNCH, PARK);
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
