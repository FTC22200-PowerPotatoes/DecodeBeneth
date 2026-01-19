package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

@Autonomous(name="DecodeAutoBlueFrontNew", group="Decode")
public class DecodeAutoBlueFrontNew extends OpMode {

    private DcMotorEx launcher;
    GoBildaPinpointDriver odo;
    Pose2D pos = null;
    double oldTime = 0;
    //Maybe adjust this based on the amount of time we have left after autonomous
    double howManyBallsToCollect;
    boolean alliance = false; // false/true → you decide which is red/blue

    private AnalogInput laserAnalog;

    // Launcher velocities
    final double LAUNCHER_TARGET_VELOCITY = 1680.0;
    final double LAUNCHER_MIN_VELOCITY = 1580.0;

    double shotsToFire = 3;
    double TIME_BETWEEN_SHOTS = 3.0;
    double boxServoTime = 0.7;
    double robotRotationAngle = 40.5;
    boolean limelightOn = false;
    boolean driveOffLine = true;

    double fL_Motor = 0.0;
    double fR_Motor = 0.0;
    double bL_Motor = 0.0;
    double bR_Motor = 0.0;
    ColorSensor colorSensor;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 4000.0;
    double volts = 0.1;
    double distanceInch = (volts / MAX_VOLTS) * (MAX_DISTANCE_MM / 25.4);

    // Motion constants
    final double DRIVE_SPEED = 0.5;
    double driveModo = 0;
    double tx;
    double ta;
    double ty;
    Pose3D botPose;
    String motiff;
    final double ROTATE_SPEED = 1.0;
    double distanceMeters;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Distances (in inches)
    final double SHOOT_POSITION_DISTANCE_IN = 48.0;
    final double RETURN_TO_MIDDLE_DISTANCE_IN = 24.0;

    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;
    private DcMotor intakeMotor = null;
    private Servo boxServo = null;
    boolean followMotiff = true;
    CRServo leftFeeder;
    CRServo rightFeeder;
    CRServo topWheel;
    Servo rgbLight;
    private Limelight3A limelight;
    private IMU imu;



    private enum LaunchState { IDLE, PREPARE, PREPARE2, LAUNCH }
    private LaunchState launchState;

    private enum AutonomousState {
        DRIVEALITTLE,
        POINT_TO_SHOOT,
        LIMELIGHT,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        RETURN_TO_MIDDLE,
        NEXTBALL,
        ROTATING,
        COMPLETE
    }
    private AutonomousState autonomousState;


    private boolean driveTargetSet = false;
    private boolean rotateTargetSet = false;

    public Pose2D autoPos;

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
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        rgbLight = hardwareMap.get(Servo.class, "RGB Light");

        // Limelight initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // Offsets (mm)
        odo.setOffsets(-22.1, -136.4, DistanceUnit.MM);
        odo.update();

        // Starting position on the field (example values)
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM,
                609.6,
                -1524.0,
                AngleUnit.DEGREES,
                0
        );
        odo.setPosition(startingPosition);

        pos = odo.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset encoders & braking
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Default to RUN_WITHOUT_ENCODER; drive() / rotate() will switch to RUN_TO_POSITION as needed
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        // Initial servo position (closed)
        boxServo.setPosition(0.85);
        telemetry.addData("Init", "Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (gamepad1.x || gamepad1.square) {
            driveOffLine = false;
        } else if (gamepad1.y || gamepad1.triangle) {
            driveOffLine = true;
        } else if (gamepad1.a || gamepad1.cross) {
            followMotiff = true;
        } else if (gamepad1.b || gamepad1.circle) {
            followMotiff = false;
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        boolean isValid = llResult != null && llResult.isValid();
        if (isValid) {
            telemetry.addLine("AprilTag Detected");
            tx = llResult.getTx();
            ty = llResult.getTy();
            ta = llResult.getTa();
            botPose = llResult.getBotpose_MT2();
            int motiffID = 21;

            telemetry.addData("PRESS X", " TO NOT DRIVE OFF THE LINE! (and PRESS Y TO DRIVE OFF LINE IF X WAS ALREADY PRESSED.)");
            telemetry.addData("Drive off line: ", driveOffLine);
            telemetry.addData("Aiden", " sucks >:(");
        }
    }

    @Override
    public void start() {
        shotsToFire = 3;
        shotTimer.reset();
        launcherSpinupTimer.reset();
        driveTimer.reset();
        boxServoTimer.reset();
    }

    @Override
    public void loop() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        telemetry.addData("State", autonomousState);
        telemetry.addData("LaunchState", launchState);

        // IMU + Limelight orientation
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // Laser distance
        volts = laserAnalog.getVoltage();
        distanceInch = (volts / MAX_VOLTS) * (MAX_DISTANCE_MM / 25.4);

        odo.update();
        pos = odo.getPosition(); // update the field, not a local variable

        telemetry.addData("Voltage (V)", "%.3f", volts);
        telemetry.addData("Laser Dist (in)", "%.2f", distanceInch);

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        String velocity = String.format(Locale.US,
                "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                odo.getVelX(DistanceUnit.MM),
                odo.getVelY(DistanceUnit.MM),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        LLResult llResult = limelight.getLatestResult();
        boolean isValid = llResult != null && llResult.isValid();
        if (isValid) {
            telemetry.addLine("AprilTag Detected");
            tx = llResult.getTx();
            ty = llResult.getTy();
            ta = llResult.getTa();
            double distanceInches;
            botPose = llResult.getBotpose_MT2();
            telemetry.addData("Motiff Pattern: ", motiff);

            double cameraHeight = 0.30;     // meters
            double cameraAngle = 20.0;      // degrees upward
            double tagHeight = 0.45;        // meters (center of tag)

            double angleToTag = Math.toRadians(cameraAngle + ty);

            distanceMeters = (tagHeight - cameraHeight) / Math.tan(angleToTag);
            distanceInches = distanceMeters * 39.37;

            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("Horizantal Offset: ", tx);
            telemetry.addData("Verticle Offset:  (in)", ty);
            telemetry.addData("Area of tag: ", distanceInches);
        } else {
            telemetry.addLine("No AprilTag Detected");
        }

        if (limelightOn && isValid) {
            // 'amt' of turn


            double offsetCAngle = Math.toDegrees(Math.atan(0.126975 / distanceMeters));

            double kP = 0.02;
            double turnPower = kP * (tx+1.0);
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
            if (Math.abs(tx) < 1.0) turnPower = 0;

            // Rotate robot via above
            fL_Motor += -turnPower;
            bL_Motor += -turnPower;
            fR_Motor += turnPower;
            bR_Motor += turnPower;

            // Telemetry for data
            telemetry.addData("Left/Right offset: ", tx);
            telemetry.addData("Turn Power: ", turnPower);
        }

        // Autonomous state machine
        switch (autonomousState) {
            case DRIVEALITTLE:
                if (drive(DRIVE_SPEED, 6.0, DistanceUnit.INCH, 0.5, 0)) {
                    autonomousState = AutonomousState.POINT_TO_SHOOT;
                }
                break;

            case POINT_TO_SHOOT:
                if (rotate(ROTATE_SPEED, 40.5, AngleUnit.DEGREES, 1.0)) {
                    // Reset encoders, then back to RUN_WITHOUT_ENCODER
                    limelightOn = true;
                    boxServoTimer.reset();
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;
            case LIMELIGHT:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (boxServoTimer.seconds() > 1.0) {
                    limelightOn = false;
                    autonomousState = AutonomousState.LAUNCH;
                }
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        launcher.setVelocity(0.0);
                        if (driveModo > 6) {
                            driveModo = 7;
                            autonomousState = AutonomousState.ROTATING;
                        } else {
                            autonomousState = AutonomousState.ROTATING;
                        }
                    }
                }
                break;

            case RETURN_TO_MIDDLE:

                if (drive(DRIVE_SPEED, 36.0, DistanceUnit.INCH, 0.5, 0)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case ROTATING:
                // Measures the heading/angle of the current robot with odometry and then rotates until it reaches zero because it negates itself.
                if (rotate(ROTATE_SPEED, -odo.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES, 0.0)) {
                    frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                    frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    if (driveModo < 6) {
                        driveModo = 0;
                    } else {
                        driveModo = 8;
                    }
                    autonomousState = AutonomousState.NEXTBALL;
                }
                break;
            case NEXTBALL:
                switch (motiff) {
                    case "PURPLE PURPLE GREEN":
                        if (drive(DRIVE_SPEED, 72.0, DistanceUnit.INCH, 0.0, 0) && (driveModo == 0)) {
                            driveModo = 1;
                        }
                        break;
                    case "PURPLE GREEN PURPLE":
                        if (drive(DRIVE_SPEED, 48.0, DistanceUnit.INCH, 0.0, 0) && driveModo == 0) {
                            driveModo = 1;
                        }
                        break;
                    default:
                        if (drive(DRIVE_SPEED, 24.0, DistanceUnit.INCH, 0.0, 0) && driveModo == 0) {
                            driveModo = 1;
                        }
                        break;
                }
                if ((driveModo == 1 || driveModo == 9) && rotate(ROTATE_SPEED, -90.0, AngleUnit.DEGREES, 0.0)) {
                    frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                    frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                    driveModo += 1;
                }
                if (drive(1.0, -54.0, DistanceUnit.INCH, 0.0, 0) && (driveModo == 2 || driveModo == 10)) {
                    driveModo += 1;
                }
                if (drive(1.0, 54.0, DistanceUnit.INCH, 0.0, 0) && (driveModo == 3 || driveModo == 11)) {
                    driveModo += 1;
                }
                if ((driveModo == 4 || driveModo == 12) && rotate(ROTATE_SPEED, 90.0, AngleUnit.DEGREES, 0.2)) {
                    frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                    frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                    driveModo += 1;
                }
                if (driveModo < 6) {
                    switch (motiff) {
                        case "PURPLE PURPLE GREEN":
                            if (drive(DRIVE_SPEED, -72.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 5) {
                                driveModo = 6;
                            }
                            break;
                        case "PURPLE GREEN PURPLE":
                            if (drive(DRIVE_SPEED, -48.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 5) {
                                driveModo = 6;
                            }
                            break;
                        default:
                            if (drive(DRIVE_SPEED, -24.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 5) {
                                driveModo = 6;
                            }
                            break;
                    }
                } else {
                    switch (motiff) {
                        case "PURPLE PURPLE GREEN":
                            if (drive(DRIVE_SPEED, -24.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 13) {
                                driveModo = 14;
                            }
                            break;
                        case "PURPLE GREEN PURPLE":
                            if (drive(DRIVE_SPEED, -24.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 13) {
                                driveModo = 14;
                            }
                            break;
                        default:
                            if (drive(DRIVE_SPEED, -48.0, DistanceUnit.INCH, 0.5, 0) && driveModo == 13) {
                                driveModo = 14;
                            }
                            break;
                    }
                }
                if ((driveModo == 6 || driveModo == 14) && rotate(ROTATE_SPEED, 40.8, AngleUnit.DEGREES, 0.0)) {

                    limelightOn = true;
                    boxServoTimer.reset();
                    driveModo += 1;
                    if (driveModo < 8) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        autonomousState = AutonomousState.RETURN_TO_MIDDLE;
                    }
                }

                switch (motiff) {
                    case "PURPLE PURPLE GREEN":
                        if (drive(DRIVE_SPEED, 24.0, DistanceUnit.INCH, 0.0, 0) && (driveModo == 8)) {
                            driveModo = 9;
                        }
                        break;
                    case "PURPLE GREEN PURPLE":
                        if (drive(DRIVE_SPEED, 24.0, DistanceUnit.INCH, 0.0, 0) && driveModo == 8) {
                            driveModo = 9;
                        }
                        break;
                    default:
                        if (drive(DRIVE_SPEED, 48.0, DistanceUnit.INCH, 0.0, 0) && driveModo == 8) {
                            driveModo = 9;
                        }
                        break;
                }
                if (driveModo == 2 || driveModo == 10) {
                    intakeMotor.setPower(1.0);
                    if (distanceInch < 500 || red < 25555 || blue < 25555 || green < 25555) {
                        leftFeeder.setPower(-1.0);
                        rightFeeder.setPower(1.0);
                        topWheel.setPower(-1.0);

                    } else {
                        leftFeeder.setPower(0.0);
                        rightFeeder.setPower(0.0);
                        topWheel.setPower(0.0);
                    }
                } else {
                    intakeMotor.setPower(0.0);
                }
                break;
            case COMPLETE:
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
                intakeMotor.setPower(0.0);
                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);
                autoPos = odo.getPosition();
                break;
        }

        telemetry.update();
    }

    // Launch routine
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

                /* Example velocity logic: close vs far
                if (distanceToGoalMm(true) / 25.4 < 80.0) {
                    launcher.setVelocity(Math.ceil(23.333 * distanceToGoalMm(alliance) / 25.4));
                } else {
                    launcher.setVelocity(Math.ceil(18.555 * distanceToGoalMm(alliance) / 25.4));
                }
*/
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Wait for velocity or timeout
                // So low because we just need to get the launcher going while the intake is running and the launcher already revs up pretty fast.
                if (launcher.getVelocity() > (LAUNCHER_MIN_VELOCITY) || launcherSpinupTimer.seconds() > 2.0) {
                    boxServo.setPosition(0.85); // open to feed
                    boxServoTimer.reset();
                    shotTimer.reset();
                    launchState = LaunchState.PREPARE2;
                }
                break;

            case PREPARE2:
                intakeMotor.setPower(1.0);
                boxServo.setPosition(0.85);
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
                if (boxServoTimer.seconds() > 2/5.0 || distanceInch > 500.0) {
                    boxServoTimer.reset();
                    shotTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (boxServoTimer.seconds() > boxServoTime) {
                    boxServo.setPosition(0.85);
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                } else {
                    intakeMotor.setPower(0.0);
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);
                    boxServo.setPosition(0.6);

                }
                break;
        }
        return false;
    }

    // Drive with run-to-position, using a "hold" time
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds, double turnPower) {
        final double TOLERANCE_MM = 10;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        if (!driveTargetSet) {
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

            double forward = Math.abs(speed);

            double leftPower  = forward - turnPower;
            double rightPower = forward + turnPower;

            // Clip to [-1, 1]
            leftPower  = Math.max(-1, Math.min(1, leftPower));
            rightPower = Math.max(-1, Math.min(1, rightPower));

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            driveTimer.reset();
            driveTargetSet = true;
        }

        if (Math.abs(targetPosition - frontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            frontLeft.setZeroPowerBehavior(BRAKE);
            backLeft.setZeroPowerBehavior(BRAKE);
            frontRight.setZeroPowerBehavior(BRAKE);
            backRight.setZeroPowerBehavior(BRAKE);
            intakeMotor.setZeroPowerBehavior(BRAKE);
            launcher.setZeroPowerBehavior(BRAKE);
            driveTimer.reset();
        } else {
            frontLeft.setZeroPowerBehavior(FLOAT);
            backLeft.setZeroPowerBehavior(FLOAT);
            frontRight.setZeroPowerBehavior(FLOAT);
            backRight.setZeroPowerBehavior(FLOAT);
            intakeMotor.setZeroPowerBehavior(FLOAT);
            launcher.setZeroPowerBehavior(FLOAT);
        }

        boolean done = (driveTimer.seconds() > holdSeconds);
        if (done) {
            driveTargetSet = false;
        }
        return done;
    }

    // Rotate with run-to-position
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        driveTimer.reset();
        if (!limelightOn) {
            double TOLERANCE_MM = 10;

            double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2.0);
            double leftTargetPosition = (targetMm * TICKS_PER_MM);
            double rightTargetPosition = -targetMm * TICKS_PER_MM;

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

            if (Math.abs(leftTargetPosition - frontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
                frontLeft.setZeroPowerBehavior(BRAKE);
                backLeft.setZeroPowerBehavior(BRAKE);
                frontRight.setZeroPowerBehavior(BRAKE);
                backRight.setZeroPowerBehavior(BRAKE);
                intakeMotor.setZeroPowerBehavior(BRAKE);
                launcher.setZeroPowerBehavior(BRAKE);
                driveTimer.reset();
            } else {
                frontLeft.setZeroPowerBehavior(FLOAT);
                backLeft.setZeroPowerBehavior(FLOAT);
                frontRight.setZeroPowerBehavior(FLOAT);
                backRight.setZeroPowerBehavior(FLOAT);
                intakeMotor.setZeroPowerBehavior(FLOAT);
                launcher.setZeroPowerBehavior(FLOAT);
                driveTimer.reset();
            }

            boolean done = (driveTimer.seconds() > holdSeconds);
            if (done) {
                rotateTargetSet = false;
            }
            return done;
        } else {
            return true;

        }
    }

    public double distanceToGoalMm(boolean isRed) {
        if (pos != null) {
            if (isRed) {
                double goalX = 73.25 * 25.4;
                double goalY = 69.15 * 25.4;
                double dx = goalX - pos.getX(DistanceUnit.MM);
                double dy = goalY - pos.getY(DistanceUnit.MM);
                return Math.hypot(dx, dy);
            } else {
                double goalX = -73.25 * 25.4;
                double goalY = -69.15 * 25.4;
                double dx = goalX - pos.getX(DistanceUnit.MM);
                double dy = goalY - pos.getY(DistanceUnit.MM);
                return Math.hypot(dx, dy);
            }
        } else {
            return 0.0;
        }
    }
}