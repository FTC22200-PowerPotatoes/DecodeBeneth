package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous
public class RedBackAutoP extends OpMode {
    private Timer pathTimer, opModeTimer;
    private Follower follower;
    private intakeOuttake take;

    public enum PathState {
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
        SHOOT_FINAL_3,
        SHOOT_3,
        MOVE_OFF_LINE,
        COMPLETED,
    }
    private PathState pathState;

    private final Pose startPose = new Pose(85, 8, Math.toRadians(90));
    /* private final Pose shootPose = new Pose(85, 10.963827304550756, Math.toRadians(62));
    private final Pose goToGrabFirst3Pose = new Pose(95.30841121495327, 35.95327102803738, Math.toRadians(180));
    private final Pose grabFirst3Pose = new Pose(125.88785046728975, 35.95327102803738, Math.toRadians(180));
    private final Pose shootFirst3Pose = new Pose(85, 10.963827304550756, Math.toRadians(62));
    private final Pose goToGrabNext3Pose = new Pose(95.30841121495327, 59.803738317757, Math.toRadians(180));
    private final Pose grabNext3Pose = new Pose(122.15887850466167, 59.803738317757, Math.toRadians(180));
    private final Pose shootNext3Pose = new Pose(85, 10.963827304550756, Math.toRadians(62));
    private final Pose goToGrabFinal3Pose = new Pose(95.30841121495327, 83.5, Math.toRadians(180));
    private final Pose grabFinal3Pose = new Pose(125, 83.5, Math.toRadians(180));
    private final Pose shootFinal3Pose = new Pose(85, 10.963827304550756, Math.toRadians(62));
    private final Pose moveOffLinePose = new Pose(120, 69.86915887850466, Math.toRadians(0)); */

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 8.000),

                                new Pose(85.000, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(62))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 10.964),

                                new Pose(95.308, 35.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.308, 35.953),

                                new Pose(125.888, 35.953)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.888, 35.953),

                                new Pose(85.000, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(62))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 10.964),

                                new Pose(95.308, 59.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.308, 59.804),

                                new Pose(122.159, 59.804)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.159, 59.804),

                                new Pose(85.000, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(62))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 10.964),

                                new Pose(95.308, 83.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.308, 83.500),

                                new Pose(125.000, 83.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 83.500),

                                new Pose(85.000, 10.964)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(62))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 10.964),

                                new Pose(120.000, 69.869)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(62), Math.toRadians(0))

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

                            setPathState(PathState.GO_TO_GRAB_FIRST_3);
                        }
                    }
                }
                break;
            case GO_TO_GRAB_FIRST_3:
                // Wait for path to complete, then shoot
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path2, true);
                    setPathState(PathState.GRAB_FIRST_3);
                }
                break;
            case GRAB_FIRST_3:
                // Start path to grab first 3 samples
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
                    take.intakeMotor.setPower(1.0);
                    take.leftFeeder.setPower(-1.0);
                    take.rightFeeder.setPower(1.0);

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.followPath(Path3, true);
                        setPathState(PathState.SHOOT_FIRST_3);
                    }
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 4.5) {
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        take.intakeMotor.setPower(1.0);
                        take.leftFeeder.setPower(-1.0);
                        take.rightFeeder.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 5.5) {
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        take.topWheel.setPower(-0.8);
                        take.boxMotor.setPower(1.0);
                        if (pathTimer.getElapsedTimeSeconds() > 7.0) {
                            take.intakeMotor.setPower(0);
                            take.leftFeeder.setPower(0);
                            take.rightFeeder.setPower(0);
                            take.topWheel.setPower(0.0);
                            take.boxMotor.setPower(0.0);

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

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    public void init() {
        // States
        pathState = PathState.DRIVE_STARTING_SHOOT_POS;
        // Pedro Pathing
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
        // Update follower and state machine
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time in seconds", pathTimer.getElapsedTimeSeconds());
    }
}