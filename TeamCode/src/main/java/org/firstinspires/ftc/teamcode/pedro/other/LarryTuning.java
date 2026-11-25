package org.firstinspires.ftc.teamcode.pedro.other;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class LarryTuning {


    @Tuner(name = "LarryBoy Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "LarryBoy Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }


    @Tuner(name = "LarryBoy Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, LarryConstants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, LarryConstants.mecanumConfig));
    }

    @Tuner(name = "LarryBoy Tests")
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, LarryConstants.mecanumConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, LarryConstants.localizerConfig),
                null);
    }
}
