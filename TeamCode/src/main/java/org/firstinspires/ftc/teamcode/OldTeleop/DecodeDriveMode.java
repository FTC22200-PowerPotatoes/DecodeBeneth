package org.firstinspires.ftc.teamcode.OldTeleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class DecodeDriveMode extends LinearOpMode {
    double launcher_velocity = 3000.0;
    boolean boxServoUp = false;
    //private AnalogInput laserAnalog;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private DcMotor intakeMotor;
    private DcMotorEx launcher;
    CRServo leftFeeder;
    CRServo rightFeeder;
    Servo rgbLight; // For color

    // HERE //
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        ColorSensor colorSensor;
        // Motor config
        //laserAnalog = hardwareMap.get(AnalogInput.class, "laserAnalogInput");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
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

        // Limelight initalization HERE!
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot (
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        final int CYCLE_MS = 5;

        waitForStart();
        if (isStopRequested()) return;

        // HERE //
        limelight.start();

        while (opModeIsActive()) {
            // HERE //
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            //double volts = laserAnalog.getVoltage();

            // Convert voltage to distance in millimeters (linear mapping)
            //double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

            // Telemetry
            //telemetry.addData("Voltage (V)", "%.3f", volts);
            //telemetry.addData("Distance (mm)", "%.1f", distanceMM);

            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            int purple = colorSensor.red() + colorSensor.blue();
            telemetry.addData("Green", green);
            telemetry.addData("Purple", purple);
            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);

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
            if (Math.abs(y) < 0.05) y= 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double fL_Motor = (y + x + rx) / denominator;
            double bL_Motor = (y - x + rx) / denominator;
            double fR_Motor = (y - x - rx) / denominator;
            double bR_Motor = (y + x - rx) / denominator;


            // Limelight alignment HERE //
            LLResult llResult = limelight.getLatestResult();
            boolean isValid = llResult != null && llResult.isValid();

            if (isValid) {
                telemetry.addLine("AprilTag Detected");
            } else {
                telemetry.addLine("No AprilTag Detected");
            }

            // Press a to turn on auto-aim
            if (gamepad1.right_trigger > 0.0 && isValid) {
                double tx = llResult.getTx();
                // 'amt' of turn
                double kP = 0.02;
                double turnPower = kP * tx;
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
            // Regular Motor Controls
            frontLeft.setPower(fL_Motor);
            backLeft.setPower(bL_Motor);
            frontRight.setPower(fR_Motor);
            backRight.setPower(bR_Motor);


            // Intake motor's control
            if (gamepad2.right_stick_y != 0.0) {
                intakeMotor.setPower(gamepad2.right_stick_y);
            } else {
                intakeMotor.setPower(0.0);
            }

            if (gamepad2.right_bumper) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(1.0);
            } else {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
            }
            if (gamepad2.y && !boxServoUp) {
                boxServo.setPosition(0.6);
                boxServoUp = true;
                boxServoTimer.reset();
            }
            // Timer
            if (boxServoUp && boxServoTimer.seconds() > 0.75) {
                boxServo.setPosition(0.85);
                boxServoUp = false;
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
            telemetry.addData("Detected Color:", detectedColor);

            // Displaying color
            if (detectedColor.equals("GREEN")) {
                rgbLight.setPosition(0.500);
            } else if (detectedColor.equals("PURPLE")) {
                rgbLight.setPosition(0.7222);
            } else {
                rgbLight.setPosition(1.0);
            }

            if (gamepad2.left_bumper) {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

            //Incremental velocity power
            if (gamepad2.left_stick_y > 0.0) {
                launcher_velocity += 100;
            } else if (gamepad2.left_stick_y < 0.0) {
                launcher_velocity -= 100;
            }

            telemetry.update();
            telemetry.addData("Launcher target velocity : ", launcher_velocity);
            telemetry.addData("Acc target velocity: ", launcher.getVelocity());
            sleep(CYCLE_MS);
            idle();
        }
    }
    public void launch() { // changed from private
        if (gamepad2.dpad_up) { // high
            launcher_velocity = 2225.0; // AIDEN sucks bad >:((
        } else if (gamepad2.dpad_left) { // medium
            launcher_velocity = 2100.0;
        } else if (gamepad2.dpad_right) { // low-mid (new)
            launcher_velocity = 1800.0;
        } else if (gamepad2.dpad_down) { // low
            launcher_velocity = 1500.0 ;
        }
        launcher.setVelocity(launcher_velocity);
    }
}
