package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class RedFrontAutoP extends OpMode {
    private Timer pathTimer, opModeTimer;
    private Follower follower;
    private intakeOuttake take;
    public enum PathState {
        // START POS - END POS
        // DRIVE -> Movement
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTING_SHOOT_POS,
        SHOOT_0,
        GRAB_FIRST_3,
        SHOOT_FIRST_3,
        SHOOT_1,
        GO_TO_GRAB_NEXT_3,
        GRAB_NEXT_3,
        SHOOT_NEXT_3,
        SHOOT_2,
        GO_TO_GRAB_FINAL_3,
        GRAB_FINAL_3,
        SHOOT_FINAL_3,
        SHOOT_3,
        MOVE_OFF_LINE,
        COMPLETED,
    }// state machine to keep track of diff paths



    PathState pathState;

    private final Pose startPose = new Pose (124.018691589, 123.51401869158879, Math.toRadians(45));

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10; // The multiple paths

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.019, 123.514),

                                new Pose(84.236, 83.411)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.236, 83.411),

                                new Pose(130.299, 83.411)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(130.299, 83.411),

                                new Pose(84.236, 83.411)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.236, 83.411),

                                new Pose(100.159, 59.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.159, 59.804),

                                new Pose(126.009, 59.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.009, 59.804),

                                new Pose(84.236, 83.411)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.236, 83.411),

                                new Pose(96.159, 35.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(110))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.159, 35.953),

                                new Pose(126.243, 35.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.243, 35.953),

                                new Pose(84.236, 83.411)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.236, 83.411),

                                new Pose(120.636, 69.869)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();
    }


    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTING_SHOOT_POS:
                follower.followPath(Path1, true); // for true, holds robot in final pos
                setPathState(PathState.SHOOT_0); // resets timer and new state onto next
                break;
            case SHOOT_0:
                if (!follower.isBusy()) {
                    take.launcher.setVelocity(1300.0);
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
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

                            setPathState(PathState.GRAB_FIRST_3);
                        }
                    }
                }
                break;
            case GRAB_FIRST_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) { // do I need isBusy?
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path2, true);
                        setPathState(PathState.SHOOT_FIRST_3);
                    }
                }
                break;
            case SHOOT_FIRST_3: // JUST POSE
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path3, true);
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case SHOOT_1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);

                            setPathState(PathState.GO_TO_GRAB_NEXT_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_NEXT_3: //
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path4, true);
                        setPathState(PathState.GRAB_NEXT_3);
                    }
                }
                break;
            case GRAB_NEXT_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(Path5, true);
                    setPathState(PathState.SHOOT_NEXT_3);
                }
                break;
            case SHOOT_NEXT_3: // JUST POSE
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path6, true);
                    setPathState(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);

                            setPathState(PathState.GO_TO_GRAB_FINAL_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_FINAL_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path7, true);
                        setPathState(PathState.GRAB_NEXT_3);
                    }
                }
                break;
            case GRAB_FINAL_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(Path8, true);
                    setPathState(PathState.SHOOT_FINAL_3);
                }
                break;
            case SHOOT_FINAL_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path9, true);
                    setPathState(PathState.SHOOT_3);
                }
                break;
            case SHOOT_3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
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
                    follower.followPath(Path10, true);
                    setPathState(PathState.COMPLETED);
                }
                break;
            case COMPLETED:
                // Turn off all machinery already done in final shoot case
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
