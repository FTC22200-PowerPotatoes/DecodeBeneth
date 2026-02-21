package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;

@Autonomous
public class BlueBackAutoP extends OpMode {

    private Timer pathTimer, opModeTimer;
    private Follower follower;
    private intakeOuttake take;

    public enum  PathState {
        // START POS - END POS
        // DRIVE -> Movement
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTING_SHOOT_POS,
        SHOOT_0,
        GO_TO_GRAB_FIRST_3,
        GRAB_FIRST_3,
        SHOOT_FIRST_3,
        SHOOT_1,
        GO_TO_GRAB_NEXT_3,
        GRAB_NEXT_3,
        SHOOT_NEXT_3,
        SHOOT_2,
        GO_TO_GRAB_FINAL_3,
        GRAB_FINAL_3,
        BACK_UP_A_BIT,
        SHOOT_FINAL_3,
        SHOOT_3,
        MOVE_OFF_LINE,
        COMPLETED,
    }// state machine to keep track of diff paths

    PathState pathState;
    private final Pose startPose = new Pose (58.85647607934656, 8, Math.toRadians(90));

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12; // The multiple paths

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                new Pose(58.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 10.964),

                                new Pose(45.243, 41.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.243, 41.953),

                                new Pose(14.645, 41.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.645, 41.935),

                                new Pose(58.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(114.5))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 10.964),

                                new Pose(45.243, 65.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.243, 65.804),

                                new Pose(19.645, 65.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.645, 65.804),

                                new Pose(58.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(113))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 10.964),

                                new Pose(45.243, 88.270)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.243, 88.270),

                                new Pose(20.645, 88.270)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20.645, 88.270),
                        new Pose(9.645, 88.270)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.645, 88.270),

                                new Pose(58.856, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(113))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.856, 10.964),

                                new Pose(24.159, 69.869)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))

                .build();
    }
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTING_SHOOT_POS:
                // First path - start directly without checking isBusy
                follower.followPath(Path1, true);
                setPathState(PathState.SHOOT_0);
                break;
            case SHOOT_0:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        take.intakeMotor.setPower(1.0);
                        take.leftFeeder.setPower(-1.0);
                        take.rightFeeder.setPower(1.0);
                        take.topWheel.setPower(-1.0);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.GO_TO_GRAB_FIRST_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_FIRST_3:
                // Wait for path to complete, then shoot
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);
                    follower.followPath(Path2, true);
                    setPathState(PathState.GRAB_FIRST_3);
                }
                break;
            case GRAB_FIRST_3:
                // Start path to grab first 3 samples
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(Path3, true);
                    setPathState(PathState.SHOOT_FIRST_3);
                }
                break;
            case SHOOT_FIRST_3:
                // Navigate back to shoot position
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path4, true);
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case SHOOT_1:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                        take.topWheel.setPower(-1.0);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 7.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.GO_TO_GRAB_NEXT_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_NEXT_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path5, true);
                        setPathState(PathState.GRAB_NEXT_3);
                    }
                }
                break;
            case GRAB_NEXT_3:
                // Grab second set of 3 artifacts
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(Path6, true);
                    setPathState(PathState.SHOOT_NEXT_3);
                }
                break;
            case SHOOT_NEXT_3:
                // Navigate back to shoot position
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path7, true);
                    setPathState(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                        take.topWheel.setPower(-1.0);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 7.75) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.GO_TO_GRAB_FINAL_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_FINAL_3:
                // Execute third shot
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path8, true);
                        setPathState(PathState.GRAB_FINAL_3);
                    }
                }
                break;
            case GRAB_FINAL_3:
                // Grab final set of 3 samples
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(Path9, true);
                    setPathState(PathState.BACK_UP_A_BIT);
                }
                break;
            case BACK_UP_A_BIT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(Path11, true);
                    setPathState(PathState.SHOOT_FINAL_3);
                }
                break;
            case SHOOT_FINAL_3:
                // Navigate back to shoot position
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path10, true);
                    setPathState(PathState.SHOOT_3);
                }
                break;
            case SHOOT_3:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1580.0);
                    // take.turnToTag(true, true);
                    if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 7.5) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);
                            take.launcher.setVelocity(0.0);

                            setPathState(PathState.MOVE_OFF_LINE);
                        }
                    }
                }
                break;
            case MOVE_OFF_LINE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path11, true);
                    setPathState(PathState.COMPLETED);
                }
                break;
            case COMPLETED:
                // stop all machinery - completed in shoot_3
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    public void setPathState(PathState newState) { // nice helper function
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void init() {
        pathState = PathState.DRIVE_STARTING_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap); // TODO IN Constants.java
        // TODO add any init (limelight, flywheel, mechanisms)

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