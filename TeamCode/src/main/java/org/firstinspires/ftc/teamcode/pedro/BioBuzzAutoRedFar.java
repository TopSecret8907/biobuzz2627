package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz Red Far", group = "BioBuzz")
public final class BioBuzzAutoRedFar extends BioBuzzAuto {
    private static final Pose START = new Pose(56, 8, Math.toRadians(90));
    private static final Pose LAUNCH = new Pose(56.3359, 21.724, Math.toRadians(180));
    private static final Pose PARK = new Pose(4.9672, 98.0221, Math.toRadians(-56.0491));

    public BioBuzzAutoRedFar() {
        super(Alliance.RED, StartPosition.FAR, START, LAUNCH, PARK);
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
