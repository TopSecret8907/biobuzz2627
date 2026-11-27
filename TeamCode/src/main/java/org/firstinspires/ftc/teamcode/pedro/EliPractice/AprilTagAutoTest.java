package org.firstinspires.ftc.teamcode.pedro.EliPractice;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import org.firstinspires.ftc.teamcode.pedro.BioBuzzMechanisms;
import org.firstinspires.ftc.teamcode.pedro.BiobuzzConstants;

public abstract class AprilTagAutoTest extends OpMode {
    // Stage budgets total 27 seconds, leaving time before AUTO ends at 30 seconds.
    private static final double APPROACH_TIMEOUT_SECONDS = 4.0;
    private static final double PARKING_TIMEOUT_SECONDS = 14.0;
    private static final double AUTO_STOP_SECONDS = 29.5;
    private static final double SPINUP_TIMEOUT_SECONDS = 5.0;
    private static final double FLYWHEEL_TOLERANCE = 75;

    // Timed feed for four preloads. Tune on the robot with a full load.
    private static final double FEED_SECONDS = 4.0;
    protected enum Alliance { RED, BLUE }
    protected enum StartPosition { FAR, NEAR }

    private enum State {
        READY(0), APPROACH(APPROACH_TIMEOUT_SECONDS), SPINUP(SPINUP_TIMEOUT_SECONDS),
        FEEDING(FEED_SECONDS), PARKING(PARKING_TIMEOUT_SECONDS),
        COMPLETE(0), PATH_TIMED_OUT(0), AUTO_TIMED_OUT(0);

        final double durationLimit;

        State(double durationLimit) {
            this.durationLimit = durationLimit;
        }
    }

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final Alliance alliance;
    private final StartPosition route;
    private final Pose startPose;
    private final Pose launchPose;
    private final Pose parkPose;
    private State state = State.READY;
    private boolean spinupTimedOut;
    private Follower follower;
    private BioBuzzMechanisms mechanisms;
    private Path parkingPath;

    protected AprilTagAutoTest(Alliance alliance, StartPosition route, Pose startPose,
                               Pose launchPose, Pose parkPose) {
        this.alliance = alliance;
        this.route = route;
        this.startPose = startPose;
        this.launchPose = launchPose;
        this.parkPose = parkPose;
    }

    protected abstract Path buildApproachPath();

    protected abstract Path buildParkingPath();

    @Override
    public void init() {
        telemetry.log().setCapacity(10);
        follower = BiobuzzConstants.create(hardwareMap);
        follower.setPose(startPose);
        mechanisms = new BioBuzzMechanisms(hardwareMap);
        mechanisms.initialize();
        mechanisms.configureForAutonomous();
        stopLauncher();
        mechanisms.stopIntake();
        reportTelemetry();
    }

    @Override
    public void init_loop() {
        follower.update();
        follower.setPose(startPose);
        reportTelemetry();
    }

    @Override
    public void start() {
        Path approachPath = buildApproachPath();
        parkingPath = buildParkingPath();
        autoTimer.reset();
        spinupTimedOut = false;
        follower.setPose(startPose);
        follower.follow(approachPath);
        enter(State.APPROACH, "Start pressed");
    }

    @Override
    public void loop() {
        // Stop outputs before the 30-second AUTO ends, including if a stage runs late.
        if (autoTimer.seconds() >= AUTO_STOP_SECONDS) {
            if (state != State.COMPLETE && state != State.PATH_TIMED_OUT
                    && state != State.AUTO_TIMED_OUT) {
                finish(State.AUTO_TIMED_OUT, "29.5 s Auto cutoff");
            }
            reportTelemetry();
            return;
        }

        follower.update();

        switch (state) {
            case APPROACH:
                if (!follower.isBusy()) {
                    stopDrive();
                    mechanisms.setFlywheelVelocity(BioBuzzMechanisms.LAUNCH_VELOCITY);
                    enter(State.SPINUP, "Approach complete");
                } else if (stateTimer.seconds() >= APPROACH_TIMEOUT_SECONDS) {
                    finish(State.PATH_TIMED_OUT, "Approach path timed out");
                }
                break;

            case SPINUP:
                double speedError = Math.abs(BioBuzzMechanisms.LAUNCH_VELOCITY
                        - mechanisms.getFlywheelVelocity());
                if (speedError <= FLYWHEEL_TOLERANCE) {
                    mechanisms.startFeedingLauncher();
                    enter(State.FEEDING, "Flywheel within tolerance");
                } else if (stateTimer.seconds() >= SPINUP_TIMEOUT_SECONDS) {
                    spinupTimedOut = true;
                    startParking("Flywheel timed out, feeding skipped");
                }
                break;

            case FEEDING:
                if (stateTimer.seconds() >= FEED_SECONDS) {
                    startParking("Feed timer complete");
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    finish(State.COMPLETE, "Parking path complete");
                } else if (stateTimer.seconds() >= PARKING_TIMEOUT_SECONDS) {
                    finish(State.PATH_TIMED_OUT, "Parking path timed out");
                }
                break;

            case READY:
            case COMPLETE:
            case PATH_TIMED_OUT:
            case AUTO_TIMED_OUT:
                break;
        }

        reportTelemetry();
    }

    private void enter(State nextState, String reason) {
        Pose pose = follower.pose();
        telemetry.log().add("%.1fs %s -> %s (%.1fs): %s @ %.1f, %.1f, %.0f deg",
                autoTimer.seconds(), state, nextState, state == State.READY ? 0 : stateTimer.seconds(),
                reason, pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        state = nextState;
        stateTimer.reset();
    }

    private void startParking(String reason) {
        stopLauncher();
        follower.follow(parkingPath);
        enter(State.PARKING, reason);
    }

    private void finish(State result, String reason) {
        stopDrive();
        stopLauncher();
        enter(result, reason);
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private void stopLauncher() {
        mechanisms.stopFeedingLauncher();
        mechanisms.stopFlywheel();
    }

    private void reportTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start relative to parking", route);
        telemetry.addData("State", state);
        telemetry.addData("Follower", "%s (busy: %b)", follower.mode(), follower.isBusy());
        if (state.durationLimit > 0) {
            telemetry.addData("Stage time (s)", "%.1f / %.1f", stateTimer.seconds(), state.durationLimit);
        }
        if (follower.following()) {
            telemetry.addData("Path progress", "%.0f%%, segment %d",
                    follower.completion() * 100, follower.pathIndex() + 1);
        }
        telemetry.addData("Launch target (in, deg)", "%.1f, %.1f, %.0f",
                launchPose.x(), launchPose.y(), Math.toDegrees(launchPose.heading()));
        telemetry.addData("Park target (in)", "%.1f, %.1f", parkPose.x(), parkPose.y());
        double elapsed = state == State.READY ? 0 : autoTimer.seconds();
        telemetry.addData("AUTO seconds remaining", "%.1f", Math.max(0, 30.0 - elapsed));
        if (spinupTimedOut) {
            telemetry.addLine("Flywheel speed timed out. Launch skipped.");
        }
        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        double flywheelVelocity = mechanisms.getFlywheelVelocity();
        telemetry.addData("Flywheel (ticks/s)", "%.0f", flywheelVelocity);
        if (state == State.SPINUP || state == State.FEEDING) {
            telemetry.addData("Flywheel target", "%d +/- %.0f",
                    BioBuzzMechanisms.LAUNCH_VELOCITY, FLYWHEEL_TOLERANCE);
            telemetry.addData("Flywheel speed error (ticks/s)", "%.0f",
                    Math.abs(BioBuzzMechanisms.LAUNCH_VELOCITY - flywheelVelocity));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            stopDrive();
        }
        if (mechanisms != null) {
            stopLauncher();
            mechanisms.stopIntake();
        }
    }
}
