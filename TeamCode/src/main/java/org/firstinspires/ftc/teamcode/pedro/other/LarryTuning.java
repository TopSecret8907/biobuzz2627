package org.firstinspires.ftc.teamcode.pedro.other;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class LarryTuning {
    @Tuner(name = "LarryBoy Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, DecodeLarryConstants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, DecodeLarryConstants.mecanumConfig));
    }

    @Tuner(name = "LarryBoy Tests")
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, DecodeLarryConstants.mecanumConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, DecodeLarryConstants.localizerConfig),
                null);
    }
}
