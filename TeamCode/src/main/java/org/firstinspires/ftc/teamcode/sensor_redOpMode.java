package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class sensor_redOpMode extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        ColorSensor colorSensor;
        // Motor config
        DcMotor leftFront = hardwareMap.dcMotor.get("frontLeft");
        DcMotor leftBack = hardwareMap.dcMotor.get("backLeft");
        DcMotor rightFront = hardwareMap.dcMotor.get("frontRight");
        DcMotor rightBack = hardwareMap.dcMotor.get("backRight");
        DcMotor linearMotor = hardwareMap.dcMotor.get("linearMotor");
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotorEx wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Servo config
        Servo boxServo = hardwareMap.get(Servo.class, "boxServo");
        Servo leftWheelServo = hardwareMap.get(Servo.class, "leftWheelServo");
        Servo rightWheelServo = hardwareMap.get(Servo.class, "rightWheelServo");

        // Set default positions
        leftWheelServo.setPosition(0.5);
        rightWheelServo.setPosition(0.5);
        boxServo.setPosition(0.87);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setDirection(DcMotor.Direction.FORWARD);


        final double INCREMENT2 = 0.1; // amount to slew servo each CYCLE_MS cycle
        final double INCREMENT = 109.5;
        final int CYCLE_MS = 50;         // period of each cycle
        final double MAX_POS2 = 0.4;       // Maximum rotational position
        final double MIN_POS2 = 1.0; // Minimum rotational position
        int Start = wristMotor.getCurrentPosition();
        int delta = 300;
        int End = Start + delta;
        double maxSafeTemperature = 75.0; // Define a maximum safe temperature
        double forr = 0;

        // Define class members
        double position1 = 1;
        double position2 = 1.0; // Start at maximum position
        boolean rampUp = true;
        boolean wristUp = false;
        boolean wristDown = false;
        boolean wristIsUp = true;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double speedMultiplier = 1.0;

            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier = 0.25; // Reduce speed by a quarter when you hold left trigger
            }
            double y = -gamepad1.left_stick_y * speedMultiplier; // For forwards/backwards movement
            double x = gamepad1.left_stick_x * 1.1 * speedMultiplier; // The 1.1 multiplier is to counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * speedMultiplier; // Turning left/right
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // Ensures motor values stay within [-1, 1]
            double fL_Motor = (y + x + rx) / denominator; // fL = leftFront
            double bL_Motor = (y - x + rx) / denominator; // bL = leftBack
            double fR_Motor = (y - x - rx) / denominator; // fR = rightFront
            double bR_Motor = (y + x - rx) / denominator; // bR = rightBack

            leftFront.setPower(fL_Motor);
            leftBack.setPower(bL_Motor);
            rightFront.setPower(fR_Motor);
            rightBack.setPower(bR_Motor);

            // Linear motor control
            if (gamepad2.right_bumper) {
                linearMotor.setPower(-0.4); // Reverse if right bumper pressed
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger)); // Forward with right trigger
            } else {
                linearMotor.setPower(0.0); // Stop linear motor if no input
            }
            if (gamepad2.y) {
              if (rampUp) {
                  boxServo.setPosition(0.7);
                  rampUp = false;
              } else {
                  boxServo.setPosition(0.2);
                  rampUp = true;
              }
            }
            while (gamepad2.y) {
                idle();
            }
            if (gamepad2.x) {
                boxServo.setPosition(0.87);
            }

            // Intake motor control
            if (gamepad2.dpad_up) {
                intakeMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }


            if (gamepad2.b) {
                wristMotor.setPower(0.5);
                wristIsUp = false;
            }
            if (gamepad2.a) {
                wristMotor.setPower(-0.5);
                wristIsUp = true;
            }
            if (gamepad2.left_bumper) {
                wristMotor.setPower(0);
            }
            if (wristIsUp && wristMotor.getCurrentPosition() < Start) {
                wristMotor.setPower(0);
            }
            if (!wristIsUp && wristMotor.getCurrentPosition() > End) {
                wristMotor.setPower(0);
            }

            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();

            String detectedColor = "UNKNOWN";
            if (red > green && red > blue) {
                detectedColor = "RED";
            } else if (blue > red && blue > green) {
                detectedColor = "BLUE";
            } else if (green > blue && green > red) {
                detectedColor = "GREEN";
            }

            if (gamepad2.left_bumper && detectedColor.equals("BLUE")) {
                leftWheelServo.setPosition(0.0); // Full forward
                rightWheelServo.setPosition(1.0);
            } else if (gamepad2.left_bumper && detectedColor.equals("UNKNOWN")) {
                leftWheelServo.setPosition(0.0); // Full forward
                rightWheelServo.setPosition(1.0);
            } else if (gamepad2.left_bumper && detectedColor.equals("YELLOW")) {
                leftWheelServo.setPosition(0.5); // Stop the servo
                rightWheelServo.setPosition(0.5);
            } else if (gamepad2.left_bumper && detectedColor.equals("RED")) {
                leftWheelServo.setPosition(0.5); // Stop the servo
                rightWheelServo.setPosition(0.5);
            } else {
                leftWheelServo.setPosition(0.5); // Full forward
                rightWheelServo.setPosition(0.5);
            }

                // Optional: Add telemetry to display servo positions
                telemetry.addData("wristMotor position", wristMotor.getCurrentPosition());
                telemetry.addData("Left Wheel Servo Position", leftWheelServo.getPosition());
                telemetry.addData("Right Wheel Servo Position", rightWheelServo.getPosition());
                telemetry.addData("box Servo Position", boxServo.getPosition());
                telemetry.addData("linearMotor position", linearMotor.getCurrentPosition());
                telemetry.addData("Ramp Up", rampUp);
                telemetry.addData("Red", red);
                telemetry.addData("Blue", blue);
                telemetry.addData("Green", green);
                telemetry.addData("detectedColor", detectedColor);
                telemetry.update();
                sleep(20);
                idle();
            }
        }
    }

