package org.firstinspires.ftc.teamcode.pedroPathing;

import android.security.keystore.BackendBusyException;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class Game1Auto extends OpMode{

    private Timer pathTimer, opModeTimer;
    private Follower follower;
    private intakeOuttake take;

    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        STRAFE,
        COMPLETE,
    }

    PathState pathState;
    private PathChain Path1, Path2;
    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 8.000),

                                new Pose(58.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(108))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 10.964),

                                new Pose(42.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(180))

                .build();
    }

    private final Pose startPose = new Pose (58.856, 8, Math.toRadians(90));
    // private final Pose shootPose = new Pose (58.856476079346564, 10.963827304550756, Math.toRadians(118));

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_TO_SHOOT:
                // First path - start directly without checking isBusy
                follower.followPath(Path1, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1580.0);
                    take.turnToTag(true, true);
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        take.intakeMotor.setPower(1.0);
                        take.leftFeeder.setPower(-1.0);
                        take.rightFeeder.setPower(1.0);
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);

                            setPathState(PathState.STRAFE);
                        }
                    }
                }
                break;
            case STRAFE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(Path2, true);
                    setPathState(PathState.COMPLETE);
                }
                break;
            case COMPLETE:
                break;
        }
    }

    public void setPathState(PathState newState) { // nice helper function
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void init() {
        pathState = PathState.DRIVE_START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        take = new intakeOuttake(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time in seconds", pathTimer.getElapsedTimeSeconds());
    }
}
