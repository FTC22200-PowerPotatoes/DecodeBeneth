package org.firstinspires.ftc.teamcode.AutoWeUsing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="CurrentRedBackPlayoff", group="Decode")
public  class DecodeAutoRedBackOld extends OpMode {
    private DcMotorEx launcher;
    // launcher velocities (tune to your hardware)
    double LAUNCHER_TARGET_VELOCITY = 2100.0;
    double LAUNCHER_MIN_VELOCITY = 2050.0;
    double timesShot = 0;
    double shotsToFire = 3.0;
    double TIME_BETWEEN_SHOTS = 3.0;    // reduced cycle time (tune)
    double boxServoTime = 0.55;          // servo dwell time (tune)
    double robotRotationAngle = -46.5;
    boolean driveOffLine = true;
    boolean limelightOn = true;
    private Limelight3A limelight;
    private IMU imu;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime strafeTimer = new ElapsedTime();
    private ElapsedTime boxServoTimer = new ElapsedTime();
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
    private Servo boxServo = null;
    CRServo leftFeeder;
    CRServo rightFeeder;
    CRServo topWheel;

    // launch state machine
    private enum LaunchState {IDLE, PREPARE, PREPARE2, LAUNCH}

    private LaunchState launchState;

    // autonomous high-level states
    private enum AutonomousState {
        DRIVEALITTLE,
        POINT_TO_SHOOT,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        RETURN_TO_MIDDLE,
        ROTATING,
        STRAFETOSIDE,
        LIMELIGHT,
        DRIVETONEXTBALLSY,
        ROTATETOBALLS,
        DRIVETONEXTBALLSX,
        DRIVEBACKBALL,
        ROTATESTRAIGHT,
        GOBACKAGAIN,
        DRIVETONEXTBALLSY2,
        ROTATETOBALLS2,
        DRIVETONEXTBALLSX2,
        DRIVEBACKBALL2,
        ROTATESTRAIGHT2,
        GOBACKAGAIN2,
        COMPLETE
    }

    private AutonomousState autonomousState;

    // flags to ensure we set run-to-position targets only once when entering a state
    private boolean driveTargetSet = false;
    private boolean rotateTargetSet = false;
    private boolean strafeTargetSet = false;

    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVEALITTLE;
        launchState = LaunchState.IDLE;

        // Hardware mapping
        laserAnalog = hardwareMap.get(AnalogInput.class, "laserAnalogInput");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // Reset encoders & braking
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

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
        volts = laserAnalog.getVoltage();
        // Convert voltage to distance in millimeters (linear mapping)
        distanceInch = ((volts / MAX_VOLTS) * MAX_DISTANCE_MM) * 25.4;
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
                if (drive(DRIVE_SPEED, 6.0, DistanceUnit.INCH, 0.0)) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    autonomousState = AutonomousState.POINT_TO_SHOOT;
                    resetDriveFlags();
                }
                break;
            case POINT_TO_SHOOT:
                // Drive forward from start to shooting spot (distance positive = forward)
                if (rotate(ROTATE_SPEED, -robotRotationAngle, AngleUnit.DEGREES, 0.0)) {
                    // prepare launcher sequence
                    if (limelightOn) {
                        boxServoTimer.reset();
                        autonomousState = AutonomousState.LIMELIGHT;
                    } else {
                        autonomousState = AutonomousState.LAUNCH;
                    }
                    resetDriveFlags();
                }
                break;
            case LIMELIGHT:
                //lumins 2
                if (rotateToTag(1.0, 1.0) || boxServoTimer.seconds() > 0.2) {
                    autonomousState = AutonomousState.LAUNCH;
                    resetDriveFlags();
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
                        shotsToFire = 3;
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

            case RETURN_TO_MIDDLE:
                // Drive BACK toward middle of field; here we use a positive distance to drive forward
                // because our drive() interprets "distance" direction consistently per call.
                if (drive(DRIVE_SPEED, 20.0, DistanceUnit.INCH, 0.0)) {
                    autonomousState = AutonomousState.ROTATETOBALLS;
                    resetDriveFlags();
                }
                break;

            case ROTATING:
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 0.0)) {
                    if (driveOffLine) {
                        if (timesShot == 0) {
                            autonomousState = AutonomousState.RETURN_TO_MIDDLE;
                        } else if (timesShot == 1) {
                            autonomousState = AutonomousState.STRAFETOSIDE;
                        } else {
                            autonomousState = AutonomousState.DRIVETONEXTBALLSY;
                        }
                    } else {
                        autonomousState = AutonomousState.COMPLETE;
                    }
                    resetDriveFlags();
                }
                break;
            case DRIVETONEXTBALLSY:
                if (drive(DRIVE_SPEED, 20.0, DistanceUnit.INCH, 0.3)) {
                    autonomousState = AutonomousState.COMPLETE;
                    resetDriveFlags();

                }
                break;
            case ROTATETOBALLS:
                if (rotate(ROTATE_SPEED, -155.0, AngleUnit.DEGREES, 0.0)) {
                    autonomousState = AutonomousState.DRIVETONEXTBALLSX;
                    resetDriveFlags();
                    timesShot = 1;
                }
                break;
            case DRIVETONEXTBALLSX:
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
                topWheel.setPower(-0.8);
                intakeMotor.setPower(1.0);
                if (drive(0.8, -48.0, DistanceUnit.INCH, 0.0)) {
                    autonomousState = AutonomousState.DRIVEBACKBALL;
                    resetDriveFlags();
                }
                break;
            case DRIVEBACKBALL:
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                if (drive(0.8, 48.0, DistanceUnit.INCH, 0.0)) {
                    intakeMotor.setPower(0.0);
                    autonomousState = AutonomousState.ROTATESTRAIGHT;
                    resetDriveFlags();
                }
                break;
            case ROTATESTRAIGHT:
                if (rotate(ROTATE_SPEED, 155.0, AngleUnit.DEGREES, 0.0)) {
                    robotRotationAngle -= 6.0;
                    autonomousState = AutonomousState.GOBACKAGAIN;
                    resetDriveFlags();
                }
                break;
            case GOBACKAGAIN:
                if (drive(DRIVE_SPEED, -20.0, DistanceUnit.INCH, 0.0)) {
                    robotRotationAngle -= 2;
                    timesShot = 1;
                    autonomousState = AutonomousState.POINT_TO_SHOOT;
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    resetDriveFlags();
                }
                break;
            case STRAFETOSIDE:
                if (strafe(DRIVE_SPEED, 30.0, DistanceUnit.INCH, 0.0)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
            case DRIVETONEXTBALLSY2:
                if (drive(DRIVE_SPEED, 46.0, DistanceUnit.INCH, 0.0)) {
                    autonomousState = AutonomousState.ROTATETOBALLS2;
                    resetDriveFlags();
                }
                break;
            case ROTATETOBALLS2:
                if (rotate(ROTATE_SPEED, -155.0, AngleUnit.DEGREES, 0.0)) {
                    autonomousState = AutonomousState.DRIVETONEXTBALLSX2;
                    resetDriveFlags();
                }
                break;
            case DRIVETONEXTBALLSX2:
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
                topWheel.setPower(-0.8);
                intakeMotor.setPower(1.0);
                if (drive(DRIVE_SPEED, -50.0, DistanceUnit.INCH, 0.0)) {
                    autonomousState = AutonomousState.DRIVEBACKBALL2;
                    resetDriveFlags();
                }
                break;
            case DRIVEBACKBALL2:
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                if (drive(0.8, 50.0, DistanceUnit.INCH, 0.0)) {
                    intakeMotor.setPower(0.0);
                    autonomousState = AutonomousState.ROTATESTRAIGHT2;
                    resetDriveFlags();
                }
                break;
            case ROTATESTRAIGHT2:
                if (rotate(ROTATE_SPEED, 155.0, AngleUnit.DEGREES, 0.0)) {
                    autonomousState = AutonomousState.GOBACKAGAIN2;
                    resetDriveFlags();
                }
                break;
            case GOBACKAGAIN2:
                if (drive(DRIVE_SPEED, -42.0, DistanceUnit.INCH, 0.0)) {
                    timesShot = 2;
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                    autonomousState = AutonomousState.POINT_TO_SHOOT;
                    resetDriveFlags();
                }
                break;
            case COMPLETE:
                // stop all motion
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                intakeMotor.setPower(0);
                launcher.setVelocity(0);
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                // nothing else to do
                break;
        }
        telemetry.addData("Launcher Velocity: ", launcher.getVelocity());
        telemetry.update();
    }

    // reset flags used for setting targets once per state
    private void resetDriveFlags() {
        driveTargetSet = false;
        rotateTargetSet = false;
        strafeTargetSet = false;
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY || launcherSpinupTimer.seconds() > 0.4) {
                    boxServo.setPosition(0.85); // open box to feed
                    boxServoTimer.reset();
                    shotTimer.reset();
                    launchState = LaunchState.PREPARE2;
                }
                break;
            case PREPARE2:
                if (shotsToFire < 3) {
                    intakeMotor.setPower(1.0);
                    boxServo.setPosition(0.85); // open box to feed
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(1.0);
                    topWheel.setPower(-1.0);
                    if (boxServoTimer.seconds() > 0.75 || distanceInch > 500.0) {
                        boxServoTimer.reset();
                        shotTimer.reset();
                        launchState = LaunchState.LAUNCH;
                    }
                    break;
                } else {
                    boxServoTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
            case LAUNCH:
                // push ball up

                if (timesShot == 2 && shotsToFire < 1) {
                    boxServo.setPosition(0.6);
                    if (drive(1.0, 12.0, DistanceUnit.INCH, 0.0)) {
                        launchState = LaunchState.IDLE;
                        autonomousState = AutonomousState.COMPLETE;
                    }
                } else {
                    if (boxServoTimer.seconds() > boxServoTime) {
                        // Open box back up after ts is shot
                        boxServo.setPosition(0.85);
                        // wait between shots
                        if (shotTimer.seconds() > 0.2) {
                            launchState = LaunchState.IDLE;
                            return true; // signal shot finished
                        }
                    } else {
                        intakeMotor.setPower(0.0);
                        leftFeeder.setPower(0.0);
                        rightFeeder.setPower(0.0);
                        boxServo.setPosition(0.6);
                        topWheel.setPower(-0.8);
                    }
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
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition((int) targetPosition);
            backLeft.setTargetPosition((int) targetPosition);
            frontRight.setTargetPosition((int) targetPosition);
            backRight.setTargetPosition((int) targetPosition);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            driveTimer.reset();
            driveTargetSet = true;
        }

        // If not at target, reset hold timer
        boolean atTarget = Math.abs(targetPosition - frontLeft.getCurrentPosition()) < (TOLERANCE_MM * TICKS_PER_MM);

        if (holdSeconds > 0) {
            if (!atTarget) {
                driveTimer.reset();
            }
            return driveTimer.seconds() > holdSeconds;
        } else {
            return atTarget;
        }
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
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            telemetry.addData("IGOTNOIDEA ", toleranceDeg);
            telemetry.update();
            return true;
        }

        // apply rotation
        frontLeft.setPower(-turn);
        backLeft.setPower(-turn);
        frontRight.setPower(turn);
        backRight.setPower(turn);

        telemetry.addData("it' s turning", turn);
        telemetry.update();

        return false;
    }

    /**
     * rotate: simple encoder-based rotate (keeps existing implementation,
     * but not used in this version; leave it for future tuning)
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetTicks = distanceUnit.toMm(distance) * TICKS_PER_MM;

        if (!strafeTargetSet) {
            // Reset encoders
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Strafing uses different wheel directions
            frontLeft.setTargetPosition((int)  targetTicks);
            backLeft.setTargetPosition((int)  -targetTicks);
            frontRight.setTargetPosition((int) -targetTicks);
            backRight.setTargetPosition((int)  targetTicks);

            // Run to position
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            strafeTimer.reset();
            strafeTargetSet = true;
        }

        // Check if all wheels are at target
        boolean flDone = Math.abs(frontLeft.getTargetPosition()  - frontLeft.getCurrentPosition())  < (TOLERANCE_MM * TICKS_PER_MM);
        boolean blDone = Math.abs(backLeft.getTargetPosition()   - backLeft.getCurrentPosition())   < (TOLERANCE_MM * TICKS_PER_MM);
        boolean frDone = Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition()) < (TOLERANCE_MM * TICKS_PER_MM);
        boolean brDone = Math.abs(backRight.getTargetPosition()  - backRight.getCurrentPosition())  < (TOLERANCE_MM * TICKS_PER_MM);

        boolean atTarget = flDone && blDone && frDone && brDone;

        if (holdSeconds > 0) {
            if (!atTarget) {
                strafeTimer.reset();
            }
            return strafeTimer.seconds() > holdSeconds;
        }

        // ⭐ Instant completion when at target
        return atTarget;
    }
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        if (!rotateTargetSet) {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition((int) leftTargetPosition);
            backLeft.setTargetPosition((int) leftTargetPosition);
            frontRight.setTargetPosition((int) rightTargetPosition);
            backRight.setTargetPosition((int) rightTargetPosition);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            driveTimer.reset();
            rotateTargetSet = true;
        }

        boolean leftAtTarget  = Math.abs(leftTargetPosition  - frontLeft.getCurrentPosition())  < (TOLERANCE_MM * TICKS_PER_MM);
        boolean rightAtTarget = Math.abs(rightTargetPosition - frontRight.getCurrentPosition()) < (TOLERANCE_MM * TICKS_PER_MM);
        boolean atTarget = leftAtTarget && rightAtTarget;

        // If holdSeconds > 0, require stability
        if (holdSeconds > 0) {
            if (!atTarget) {
                driveTimer.reset();
            }
            return driveTimer.seconds() > holdSeconds;
        }
        return atTarget;
    }

}