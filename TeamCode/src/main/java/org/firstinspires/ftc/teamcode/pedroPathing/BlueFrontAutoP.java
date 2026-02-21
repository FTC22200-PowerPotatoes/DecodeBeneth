package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueFrontAutoP extends OpMode {

    private Timer pathTimer, opModeTimer;
    private Follower follower;
    double shotsToFire = 3.0;
    double shotsFired = 0;
    boolean isShotRequested = false;
    boolean isShotComplete = false;
    private DcMotorEx launcher;
    // launcher velocities (tune to your hardware)
    double LAUNCHER_TARGET_VELOCITY = 1600.0;
    double LAUNCHER_MIN_VELOCITY = 1580.0;
    double timesShot = 0;
    double TIME_BETWEEN_SHOTS = 3.0;    // reduced cycle time (tune)
    double robotRotationAngle = -46.5;
    boolean driveOffLine = true;
    boolean limelightOn = true;
    private Limelight3A limelight;
    private IMU imu;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime strafeTimer = new ElapsedTime();
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();

    // motion constants (tune these distances to match your robot and field)
    final double DRIVE_SPEED = 1.0;
    final double ROTATE_SPEED = 1.0;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    private AnalogInput laserAnalog;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 4000.0;
    double volts = 0.0;
    double distanceInch = 158.0;

    // Distances (in inches) — TUNE these on your field:
    // Distance from starting position (front of red basket) to the shoot spot (edge where lines meet)
    final double SHOOT_POSITION_DISTANCE_IN = 48.0;      // <-- tune this to place robot at the meeting point of lines
    // Distance to drive BACK into the middle of the field after shooting
    final double RETURN_TO_MIDDLE_DISTANCE_IN = 24.0;    // <-- tune to move into middle of field

    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    boolean isValid = false;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor boxMotor;
    CRServo leftFeeder;
    CRServo rightFeeder;
    CRServo topWheel;

    public enum PathState { // pedro path stating
        // START POS - END POS
        // DRIVE -> Movement
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTING_SHOOT_POS, // Position -> Position
        SHOOT_PRELOAD, // Shooting
        GRAB_FIRST_3, // Position -> Position
        SHOOT_FIRST_3, // Position -> Position
        SHOOT_LOAD1, // Shooting
        GRAB_NEXT_3, // Position -> Position
        SHOOT_NEXT_3, // Position -> Position
        SHOOT_LOAD2, // Shooting
        GRAB_FINAL_3, // Position -> Position
        SHOOT_FINAL_3, // Position -> Position
        SHOOT_LOAD3, // Shooting
        MOVE_OFF_LINE, // Position -> Position
        COMPLETED, // All machinery off
    }// state machine to keep track of diff paths

    public enum LaunchState {
        IDLE,
        PREPARE,
        PREPARE2,
        LAUNCH,
        RESET,
    }
    LaunchState launchState;
    PathState pathState;

    // Using visualizer.pedropathing.com
    // All values are in inches
    private final Pose startPose = new Pose(20.336448598130843, 123.51401869158879, Math.toRadians(138));
    private final Pose shootPose = new Pose(56.63551401869158, 86.69158878504673, Math.toRadians(138));
    private final Pose grabFirst3Pose = new Pose (18.53271028037381, 83.41121495327099, Math.toRadians(360));
    private final Pose shootFirst3Pose = new Pose (56.63551401869158, 86.69158878504673, Math.toRadians(138));
    private final Pose goToGrabPoseNext3 = new Pose (45.2429906542056, 59.803738317757, Math.toRadians(0));
    private final Pose grabNext3Pose = new Pose (20.663551401869167, 59.97196261682244, Math.toRadians(0));
    private final Pose shootNext3Pose = new Pose (56.63551401869158, 86.69158878504673, Math.toRadians(138));
    private final Pose goToGrabFinal3Pose = new Pose (45.2429906542056, 35.95327102803738, Math.toRadians(0));
    private final Pose grabFinal3Pose = new Pose (13.644859813084107, 35.78504672897195, Math.toRadians(0));
    private final Pose shootFinal3Pose = new Pose (56.63551401869158, 86.69158878504673, Math.toRadians(138));
    private final Pose moveOffLinePose = new Pose (24.158878504672906, 69.86915887850466, Math.toRadians(180));

    private PathChain startToShoot, grabFirst3, shootFirst3, grabNext3, shootNext3, grabFinal3, shootFinal3, moveOffLine; // The multiple paths
    public void buildPaths() {
        // put in coordinates for starting pos, then ending pos (one path)
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        grabFirst3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, grabFirst3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), grabFirst3Pose.getHeading())
                .build();
        shootFirst3 = follower.pathBuilder()
                .addPath(new BezierLine(grabFirst3Pose, shootFirst3Pose))
                .setLinearHeadingInterpolation(grabFirst3Pose.getHeading(), shootFirst3Pose.getHeading())
                .build();
        grabNext3 = follower.pathBuilder() // This one has two paths
                .addPath(new BezierLine(shootFirst3Pose, goToGrabPoseNext3))
                .addPath(new BezierLine(goToGrabPoseNext3, grabNext3Pose))
                .setLinearHeadingInterpolation(shootFirst3Pose.getHeading(), goToGrabPoseNext3.getHeading())
                .setLinearHeadingInterpolation(goToGrabPoseNext3.getHeading(), grabNext3Pose.getHeading())
                .build();
        shootNext3 = follower.pathBuilder()
                .addPath(new BezierLine(grabNext3Pose, shootNext3Pose))
                .setLinearHeadingInterpolation(grabNext3Pose.getHeading(), shootNext3Pose.getHeading())
                .build();
        grabFinal3 = follower.pathBuilder() // This one also hold two paths
                .addPath(new BezierLine(shootNext3Pose, goToGrabFinal3Pose))
                .addPath(new BezierLine(goToGrabFinal3Pose, grabFinal3Pose))
                .setLinearHeadingInterpolation(shootNext3Pose.getHeading(), goToGrabFinal3Pose.getHeading())
                .setLinearHeadingInterpolation(goToGrabFinal3Pose.getHeading(), grabFinal3Pose.getHeading())
                .build();
        shootFinal3 = follower.pathBuilder()
                .addPath(new BezierLine(grabFinal3Pose, shootFinal3Pose))
                .setLinearHeadingInterpolation(grabFinal3Pose.getHeading(), shootFinal3Pose.getHeading())
                .build();
        moveOffLine = follower.pathBuilder()
                .addPath(new BezierLine(shootFinal3Pose, moveOffLinePose))
                .setLinearHeadingInterpolation(shootFinal3Pose.getHeading(), moveOffLinePose.getHeading())
                .build();
    }

    public void shooterFunctionUpdate() {
        switch(launchState) {
            case IDLE:
                isShotComplete = false;
                if (isShotRequested) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    shotsFired = 0;
                    launchState = LaunchState.PREPARE;
                }
                break;
            case PREPARE:
                // spin up launcher
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY || launcherSpinupTimer.seconds() > 0.4) {
                    boxMotor.setPower(1.0);
                    shotTimer.reset();
                    launchState = LaunchState.PREPARE2;
                }
                break;
            case PREPARE2:
                if (shotsToFire > 3) {
                    intakeMotor.setPower(1.0);
                    topWheel.setPower(-0.8);
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(1.0);
                    if (distanceInch > 500.0) {
                        shotTimer.reset();
                        launchState = LaunchState.LAUNCH;
                    }
                    break;
                } else {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                boxMotor.setPower(1.0);
                intakeMotor.setPower(1.0);

                // TODO
        }
    }
    public void statePathUpdate() { // TODO GO OVER CASES
        switch(pathState) {
            case DRIVE_STARTING_SHOOT_POS:
                follower.followPath(startToShoot, true); // for true, holds robot in final pos
                setPathState(PathState.GRAB_FIRST_3); // resets timer and new state onto next
                break;
            case GRAB_FIRST_3: // TODO need code for intaking (all time)
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) { // do I need isBusy?
                    follower.followPath(grabFirst3, true);
                    setPathState(PathState.SHOOT_FIRST_3);
                }
                break;
            case SHOOT_FIRST_3: // TODO need code for shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(shootFirst3, true);
                    setPathState(PathState.GRAB_NEXT_3);
                }
                break;
            case GRAB_NEXT_3: // TODO need code for intaking (all time)
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(grabNext3, true);
                    setPathState(PathState.SHOOT_NEXT_3);
                }
                break;
            case SHOOT_NEXT_3: // TODO need code for shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(shootNext3, true);
                    setPathState(PathState.GRAB_FINAL_3);
                }
                break;
            case GRAB_FINAL_3: // TODO need code for intaking (all time)
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(grabFinal3, true);

                    setPathState(PathState.SHOOT_FINAL_3);
                }
                break;
            case SHOOT_FINAL_3: // TODO need code for shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(shootFinal3, true);
                    setPathState(PathState.MOVE_OFF_LINE);
                    break;
                }
                break;
            case MOVE_OFF_LINE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(moveOffLine, true);
                    setPathState(PathState.COMPLETED);
                }
                break;
            case COMPLETED:
                // Turn off all machinery
                intakeMotor.setPower(0.0);
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                launcher.setPower(0.0);
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

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            driveOffLine = false;
        } else if (gamepad1.y) {
            driveOffLine = true;
        } else if (gamepad1.dpad_left) {
            limelightOn = false;
        }
        telemetry.addData("PRESS X", " TO NOT DRIVE OFF THE LINE! (and PRESS Y TO DRIVE OFF LINE IF X WAS ALREADY PRESSED.");
        telemetry.addData("Drive off line: ", driveOffLine);
        telemetry.addData("Aiden", " sucks >:(");
    }

    public void init() {
        pathState = pathState.DRIVE_STARTING_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap); // TODO IN Constants.java
        // TODO add any init (limelight, flywheel, mechanisms)

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

