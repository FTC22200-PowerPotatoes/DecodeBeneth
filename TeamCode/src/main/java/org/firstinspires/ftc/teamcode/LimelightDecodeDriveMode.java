package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

// For limelight
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp
public class LimelightDecodeDriveMode extends LinearOpMode {
    double launcher_velocity = 3000.0;
    GoBildaPinpointDriver odo;
    public Pose2D autoPos;
    Pose2D pos = null;
    double oldTime = 0;
    boolean alliance = true;
    boolean boxServoUp = false;
    int motiffID = 20;
    private AnalogInput laserAnalog;
    public double highVelocity = 1500.0;
    public double lowVelocity = 900.0;
    double curTargetVelocity = highVelocity;
    double F = 14.8;
    double P = 430.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    private double distance;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 4000.0;
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private DcMotor intakeMotor;
    private DcMotorEx launcher;
    private DcMotor boxMotor;
    CRServo leftFeeder;
    CRServo rightFeeder;
    CRServo topWheel;
    Servo rgbLight; // For color

    // LIMELIGHT //
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        ColorSensor colorSensor;
        // Motor config`

        laserAnalog = hardwareMap.get(AnalogInput.class, "laserAnalogInput");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        boxMotor = hardwareMap.get(DcMotorEx.class, "boxMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        rgbLight = hardwareMap.get(Servo.class, "RGB Light");

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Limelight initalization HERE!
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Type of odometry arm that the robot is using.
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        //Direction
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

       /*
       Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
       The IMU will automatically calibrate when first powered on, but recalibrating before running
       the robot is a good idea to ensure that the calibration is "good".
       resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
       This is recommended before you run your autonomous, as a bad initial calibration can cause
       an incorrect starting value for x, y, and heading.
        */

        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        // Measure in milimeters at the meeting, this is not currently accurate.
        odo.setOffsets(-22.1, -136.4, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.update();
        pos = odo.getPosition();
        odo.setPosition(new Pose2D(DistanceUnit.MM,
                609.6,
                -1828.8,
                AngleUnit.RADIANS,
                0
        ));

        telemetry.addData("Autoposition: ", autoPos);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        final int CYCLE_MS = 5;

        waitForStart();
        if (isStopRequested()) return;

        // HERE //
        limelight.start();

        while (opModeIsActive()) {
            // HERE //
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            double volts = laserAnalog.getVoltage();

            // Convert voltage to distance in millimeters (linear mapping)
            double distanceInch = ((volts / MAX_VOLTS) * MAX_DISTANCE_MM) * 25.4;

            odo.update();

            // Telemetry
            telemetry.addData("Voltage (V)", "%.3f", volts);
            telemetry.addData("Distance (mm)", "%.1f", distanceInch);

            if (gamepad1.x) {
                alliance = false;
            } else if (gamepad1.b) {
                alliance = true;
            }

            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            int purple = colorSensor.red() + colorSensor.blue();
            telemetry.addData("Green", green);
            telemetry.addData("Purple", purple);
            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            // PIDF Rules

           /*
           gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
            */
            pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

           /*
           gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            */

            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            if (gamepad2.dpad_up) { // high
                double distanceBased = calculateRPM(distance);
                launcher_velocity = distanceBased;
            } else if (gamepad2.dpad_left) { // medium
                launcher_velocity = 1650.0;
            } else if (gamepad2.dpad_right) { // low-mid (new)
                launcher_velocity = 1375.0;
            } else if (gamepad2.dpad_down) { // low
                launcher_velocity = 1200.0;
            }

            if (gamepad2.right_trigger > 0) {
                launch();
            } else if (gamepad2.left_trigger > 0) {
                launcher.setPower(-1.0);
            } else {
                launcher.setPower(0.0);
            }

            double speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.4; // Original - prev was 0.5
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

            // Drive calculations
            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier;
            double rx = -gamepad1.right_stick_x * speedMultiplier;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // To decrease 'noise' via small movements
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double fL_Motor = (y + x + rx) / denominator;
            double bL_Motor = (y - x + rx) / denominator;
            double fR_Motor = (y - x - rx) / denominator;
            double bR_Motor = (y + x - rx) / denominator;

            // Limelight alignment HERE //
            LLResult llResult = limelight.getLatestResult();
            boolean isValid = llResult != null && llResult.isValid();

            if (gamepad1.y) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else {
                    curTargetVelocity = highVelocity;
                }
            }

            // PID STUFF
            if (gamepad1.b) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad1.dpad_right) {
                F += stepSizes[stepIndex];
            }
            if (gamepad1.dpad_left) {
                F -= stepSizes[stepIndex];
            }
            if (gamepad1.dpad_up) {
                P += stepSizes[stepIndex];
            }
            if (gamepad1.dpad_down) {
                P -= stepSizes[stepIndex];
            }

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            double curVelocity = launcher.getVelocity();
            double error = curTargetVelocity - curVelocity;



            if (isValid) {
                Pose3D botpose = llResult.getBotpose_MT2();
                distance = getDistance(llResult.getTy());
                for (LLResultTypes.FiducialResult fid : llResult.getFiducialResults()) {
                    motiffID = fid.getFiducialId();
                }
                telemetry.addData("Distance", distance);
                telemetry.addData("LL Timestamp", llResult.getTimestamp());
                telemetry.addLine("AprilTag Detected");
            } else {
                telemetry.addLine("No AprilTag Detected");
            }

            // Press a to turn on auto-aim (limelight)
            if (gamepad1.right_trigger > 0.0 && isValid && (motiffID == 20 || motiffID == 24)) {
                // just for telemetry
                double ty = llResult.getTy();
                // 'amt' of turn
                double tx = llResult.getTx();
                double kP = 0.02;
                double turnPower = kP * (tx+3);
                turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
                if (Math.abs(tx) < 1.0) turnPower = 0;

                // Rotate robot via above
                fL_Motor += -turnPower;
                bL_Motor += -turnPower;
                fR_Motor += turnPower;
                bR_Motor += turnPower;

                // Telemetry for data
                telemetry.addData("Left/Right offset: ", tx);
                telemetry.addData("Vertical Offset ", ty);
                telemetry.addData("Turn Power: ", turnPower);
            }

            // Regular Motor Controls
            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);

            if (gamepad2.b) {
                if (distanceToGoalMm(true) / 25.4 < 80.0) {
                    launcher_velocity = Math.ceil(25.333 * distanceToGoalMm(alliance)/25.4);
                } else {
                    launcher_velocity = Math.ceil(22.555 * distanceToGoalMm(alliance)/25.4);
                }
            }

            // Intake motor's control
            if (gamepad2.right_stick_y > 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
            } else if (gamepad2.right_stick_y < 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }

            /*if (gamepad2.right_bumper) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
                topWheel.setPower(-1.0);
            } else if (gamepad2.x) {
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(-1.0);
                topWheel.setPower(1.0);
            } else {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                topWheel.setPower(0.0);
            }
            if (gamepad2.y && !boxServoUp) {
                boxServo.setPosition(0.6);
                boxServoUp = true;
                boxServoTimer.reset();
            }*/

            // Timer
            if (boxServoUp && boxServoTimer.seconds() > 0.75) {
                boxServo.setPosition(0.85);
                boxServoUp = false;
            }

            if (gamepad2.right_bumper) {
                boxMotor.setPower(1.0);
                telemetry.addData("Should be running", boxMotor.getPowerFloat());
                topWheel.setPower(-1.0);
            } else if (gamepad2.x) {
                boxMotor.setPower(-1.0);
                topWheel.setPower(1.0);
            } else {
                boxMotor.setPower(0.0);
                topWheel.setPower(0.0);
            }

            // Detected color through sensor
            String detectedColor = "UNKNOWN";
            if (red > green && red > blue || green < 250 && purple < 250) {
                detectedColor = "WHITE";
            } else if (green > blue && green > red) {
                detectedColor = "GREEN";
            } else if (green < purple) {
                detectedColor = "PURPLE";
            }

            // telemetry.addData("Detected Color:", detectedColor);

            // Displaying color
            if (detectedColor.equals("GREEN")) {
                rgbLight.setPosition(0.500);
            } else if (detectedColor.equals("PURPLE")) {
                rgbLight.setPosition(0.7222);
            } else {
                rgbLight.setPosition(1.0);
            }

            //Incremental velocity power
            if (gamepad2.left_stick_y > 0.0) {
                launcher_velocity += 100;
            } else if (gamepad2.left_stick_y < 0.0) {
                launcher_velocity -= 100;
            }

           /*
           Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
           READY: the device is working as normal
           CALIBRATING: the device is calibrating and outputs are put on hold
           NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
           FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
           FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
           FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
           FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
           */

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Distance to the red goal in mm", distanceToGoalMm(true));
            telemetry.addData("Distance to the blue goal in mm", distanceToGoalMm(false));
            telemetry.addData("Alliance selected goal: ", distanceToGoalMm(alliance));
            telemetry.addData("Alliance selected: ", alliance);
            telemetry.addData("Position on the field measured by inches: ", odo.getPosition());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

            telemetry.update();
            telemetry.addData("Launcher target velocity : ", launcher_velocity);
            telemetry.addData("Acc target velocity: ", launcher.getVelocity());
            sleep(CYCLE_MS);
            idle();
        }
    }

    /**
     *
     * @param ty - vertical offset from Limelight (degrees)
     * @return distance in inches
     */
    // Dual crosshair values
    // crosshairA: -0.28 (-0.2779847979545593), 0.74 (0.7435855269432068)
    // crosshairB: -0.12 (-0.1181691512465477), 0.21 (0.20835214853286743)

    private double getDistance(double ty) { // code from Limelight docs
        // All height in cm and angles in radians
        double camMountAngle = Math.toRadians(5.0); // TODO
        double camHeightFromGround = 34.0; // TODO Center of cam in cm
        double goalHeight = 75.0; // TODO center of tag in cm

        double angleToGoalDegrees = camMountAngle + ty;
        double inRadians = Math.toRadians(angleToGoalDegrees);
        return (goalHeight - camHeightFromGround) / Math.tan(inRadians);
    }

    //42.5
    //45
    private double calculateRPM(double distance) {
        // needs to be tweaked
        double h = 0.86;
        double angle = 45;
        double d = distance/100;
        double V_exit = Math.sqrt((9.8 * Math.pow(d, 2))/(2 * Math.pow(Math.cos(angle), 2)*(d * Math.tan(angle) - h)));
        double final_velocity = 0.0;
        double radius = 0.048;
        if (d < 250) {
            final_velocity =  (60 / (2 * Math.PI * radius)) * (V_exit / 0.799);
        } else if (d > 250) {
            final_velocity =  (60 / (2 * Math.PI * radius)) * (V_exit / 0.78);
        }
        return  final_velocity;
    }

    public void launch() { // changed from private
        launcher.setVelocity(launcher_velocity);
    }
    public double distanceToGoalMm(boolean isRed) {
        if (pos != null) {
            if (isRed) {
                double goalX = 73.25 * 25.4; // 60 in → mm
                double goalY = 69.15 * 25.4;
                double dx = goalX - pos.getX(DistanceUnit.MM);
                double dy = goalY - pos.getY(DistanceUnit.MM);
                return Math.hypot(dx, dy); // mm
            } else {
                double goalX = -73.25 * 25.4;
                double goalY = -69.15 * 25.4;
                double dx = goalX - pos.getX(DistanceUnit.MM);
                double dy = goalY - pos.getY(DistanceUnit.MM);
                return Math.hypot(dx, dy); // mm
            }
        } else {
            return 0;
        }
    }
}