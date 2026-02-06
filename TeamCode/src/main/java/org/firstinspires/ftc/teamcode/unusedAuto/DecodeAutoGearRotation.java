package org.firstinspires.ftc.teamcode.unusedAuto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="DecodeAutoGearRotation", group="Decode")
public  class DecodeAutoGearRotation extends OpMode {
    private DcMotorEx launcher;
    // launcher velocities (tune to your hardware)
    double LAUNCHER_TARGET_VELOCITY = 2100.0;
    double LAUNCHER_MIN_VELOCITY = 2050.0;

    double shotsToFire = 3.0;
    double TIME_BETWEEN_SHOTS = 3.0;    // reduced cycle time (tune)
    double boxServoTime = 0.7;          // servo dwell time (tune)
    double robotRotationAngle = -40.5;
    boolean driveOffLine = true;
    boolean limelightOn = true;
    private Limelight3A limelight;
    private IMU imu;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();

    // motion constants (tune these distances to match your robot and field)
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 1.0;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Distances (in inches) — TUNE these on your field:
    // Distance from starting position (front of red basket) to the shoot spot (edge where lines meet)
    final double SHOOT_POSITION_DISTANCE_IN = 48.0;      // <-- tune this to place robot at the meeting point of lines
    // Distance to drive BACK into the middle of the field after shooting
    final double GOBACK_DISTANCE_IN = 24.0;    // <-- tune to move into middle of field

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    boolean isValid = false;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private DcMotor intakeMotor = null;
    private Servo boxServo = null;
    CRServo leftFeeder;
    CRServo rightFeeder;
    CRServo topWheel;

    // launch state machine
    private enum LaunchState { IDLE, PREPARE, PREPARE2, LAUNCH }
    private LaunchState launchState;

    // autonomous high-level states
    private enum AutonomousState {
        DRIVEALITTLE,
        POINT_TO_SHOOT,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        GOBACK,
        ROTATING,
        LIMELIGHT,
        DRIVETONEXTBALLSY,
        ROTATETOBALLS,
        DRIVETONEXTBALLSX,
        COMPLETE
    }
    private AutonomousState autonomousState;

    // flags to ensure we set run-to-position targets only once when entering a state
    private boolean driveTargetSet = false;
    private boolean rotateTargetSet = false;

    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVEALITTLE;
        launchState = LaunchState.IDLE;

        // Hardware mapping
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");

        // Motor directions
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // Reset encoders & braking
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));

        // initial servo position (closed)
        boxServo.setPosition(0.85);
        telemetry.addData("Init", "Complete");
        // Limelight initalization HERE!
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();


        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));


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

    @Override
    public void start() {
        // reset shot counter each match start
        shotsToFire = 3;
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("State", autonomousState);
        telemetry.addData("LaunchState", launchState);
        // Limelight alignment HERE //
        LLResult llResult = limelight.getLatestResult();
        isValid = llResult != null && llResult.isValid();


        if (isValid) {
            telemetry.addLine("AprilTag Detected");
        } else {
            telemetry.addLine("No AprilTag Detected");
        }
        telemetry.addData("autoState: ", autonomousState);
        switch (autonomousState) {
            case DRIVEALITTLE:
                if (drive(DRIVE_SPEED, 6.0, DistanceUnit.INCH, 0.2)) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.POINT_TO_SHOOT;
                }
                break;
            case POINT_TO_SHOOT:
                // Drive forward from start to shooting spot (distance positive = forward)
                if(rotate(ROTATE_SPEED, -robotRotationAngle, AngleUnit.DEGREES,1)){
                    leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    resetDriveFlags();
                    // prepare launcher sequence
                    if (limelightOn) {
                        autonomousState = AutonomousState.LIMELIGHT;
                        boxServoTimer.reset();
                    } else {
                        autonomousState = AutonomousState.LAUNCH;
                    }
                }
                break;
            case LIMELIGHT:

                if (rotateToTag(1.0, 1.0) || boxServoTimer.seconds() > 0.5) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;
            case LAUNCH:
                // start shot
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        // finished firing all shots; stop launcher and drive back to middle
                        launcher.setVelocity(0);
                        if (driveOffLine) {
                            autonomousState = AutonomousState.ROTATING;
                        } else {
                            autonomousState = AutonomousState.COMPLETE;
                        }
                        resetDriveFlags();
                    }
                }
                break;

            case GOBACK:
                // Drive BACK toward middle of field; here we use a positive distance to drive forward
                // because our drive() interprets "distance" direction consistently per call.
                if (drive(DRIVE_SPEED, -6.0, DistanceUnit.INCH, 0.2)) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.ROTATETOBALLS;
                }
                break;

            case ROTATING:
                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,0.2)){
                    leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    resetDriveFlags();
                    if (driveOffLine) {
                        autonomousState = AutonomousState.GOBACK;
                    } else {
                        autonomousState = AutonomousState.COMPLETE;
                    }
                }
                break;
            case DRIVETONEXTBALLSY:
                if (drive(DRIVE_SPEED, 26.0, DistanceUnit.INCH, 0.2)) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.ROTATETOBALLS;
                }
                break;
            case ROTATETOBALLS:
                if(rotate(ROTATE_SPEED, -155.0, AngleUnit.DEGREES,0.2)) {
                    leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    resetDriveFlags();
                    autonomousState = AutonomousState.DRIVETONEXTBALLSX;
                }
                break;
            case DRIVETONEXTBALLSX:
                if (drive(1.0, -36.0, DistanceUnit.INCH, 0.2)) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                // stop all motion
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                intakeMotor.setPower(0);
                launcher.setVelocity(0);
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                // nothing else to do
                break;
        }
    }

    // reset flags used for setting targets once per state
    private void resetDriveFlags() {
        driveTargetSet = false;
        rotateTargetSet = false;
        driveTimer.reset();
    }

    // Launch routine: request shotRequested=true one time to start a shot
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    launcherSpinupTimer.reset();
                    shotTimer.reset();
                }
                break;

            case PREPARE:
                // Spin up launcher
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Wait for either sufficient velocity OR a short timeout (failsafe)

                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY || launcherSpinupTimer.seconds() > 2.0) {
                    boxServo.setPosition(0.85); // open box to feed
                    boxServoTimer.reset();
                    shotTimer.reset();
                    launchState = LaunchState.PREPARE2;
                }
                break;
            case PREPARE2:
                intakeMotor.setPower(1.0);
                boxServo.setPosition(0.85); // open box to feed
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
                topWheel.setPower(-1.0);
                if (boxServoTimer.seconds() > 1.0) {
                    boxServoTimer.reset();
                    shotTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                // push ball up

                if (boxServoTimer.seconds() > boxServoTime) {
                    // Open box back up after ts is shot
                    boxServo.setPosition(0.85);
                    // wait between shots
                    if (shotTimer.seconds() > 0.5) {
                        launchState = LaunchState.IDLE;
                        return true; // signal shot finished
                    }
                } else {
                    intakeMotor.setPower(0.0);
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    boxServo.setPosition(0.6);
                    topWheel.setPower(0.0);
                }
                break;
        }
        return false;
    }


    // Drive: improved to set targets only once per entry to state
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        if (!driveTargetSet) {
            // reset encoders so target is relative to current pose
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setTargetPosition((int) targetPosition);
            leftBack.setTargetPosition((int) targetPosition);
            rightFront.setTargetPosition((int) targetPosition);
            rightBack.setTargetPosition((int) targetPosition);

            leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            driveTimer.reset();
            driveTargetSet = true;
        }

        // If not at target, reset hold timer
        if (Math.abs(targetPosition - leftFront.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
    boolean rotateToTag(double maxPower, double toleranceDeg) {
        LLResult llResult = limelight.getLatestResult();
        telemetry.addData("heheheheehe", llResult);
        if (llResult == null || !llResult.isValid()) {
            return false;   // no tag → keep trying
        }

        telemetry.addData("dthanks what's the isValid", isValid);
        telemetry.update();
        double tx = llResult.getTx();   // horizontal offset
        double kP = 0.02;
        double turn = kP * tx;

        // clamp turn power
        turn = Math.max(-maxPower, Math.min(maxPower, turn));

        // stop if centered
        if (Math.abs(tx) < toleranceDeg) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("IGOTNOIDEA ", toleranceDeg);
            telemetry.update();
            return true;
        }

        // apply rotation
        leftFront.setPower(-turn);
        leftBack.setPower(-turn);
        rightFront.setPower(turn);
        rightBack.setPower(turn);

        telemetry.addData("it' s turning", turn);
        telemetry.update();

        return false;
    }

    /**
     * rotate: simple encoder-based rotate (keeps existing implementation,
     * but not used in this version; leave it for future tuning)
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        if (!rotateTargetSet) {
            leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setTargetPosition((int) leftTargetPosition);
            leftBack.setTargetPosition((int) leftTargetPosition);
            rightFront.setTargetPosition((int) rightTargetPosition);
            rightBack.setTargetPosition((int) rightTargetPosition);

            leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            driveTimer.reset();
            rotateTargetSet = true;
        }

        if ((Math.abs(leftTargetPosition - leftFront.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}
