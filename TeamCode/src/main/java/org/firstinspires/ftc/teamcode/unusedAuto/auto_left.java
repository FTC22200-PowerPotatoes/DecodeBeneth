package org.firstinspires.ftc.teamcode.unusedAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_left")
public class auto_left extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor linearMotor = null;
    private DcMotor intakeMotor = null;
    private Servo boxServo = null;
    private Servo wristServo = null;
    private Servo intakeServo = null;

    @Override
    public void runOpMode() {
        // Initialize the motors
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Set zero power behavior for linear motor
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Move forward
            setMotorPower(0.5, 0.5, 0.5, 0.5);
            sleep(300); // Move forward for 0.3 seconds

            // Step 2: Move left
            setMotorPower(-0.5, 0.5, 0.5, -0.5);
            sleep(1900); // Move left for 1.5 seconds

            // Spin to face the basket
            setMotorPower(-0.25, 0.25, -0.25, 0.25);
            sleep(1050); // Spin for 1.05 seconds

            // Step 3: Move back
            setMotorPower(-0.5, -0.5, -0.5, -0.5);
            sleep(295); // Move back for 0.2 seconds


            setMotorPower(0,0,0,0);

            // Operate the linear motor and servo
            linearMotor.setPower(0.5); // Extend linear motor
            boxServo.setPosition(1.0);
            sleep(2500); // Wait for 1 second

            linearMotor.setPower(0.1); // Retract linear motor
            boxServo.setPosition(0.4);
            sleep(2000);

            setMotorPower(0.5, 0.5, 0.5, 0.5);
            sleep(150); // Move forward for 0.1 seconds

            setMotorPower(0,0,0,0);

            linearMotor.setPower(-0.4);
            boxServo.setPosition(0.9); // Move box servo to position 0.4
            sleep(2000); // Wait for 1 second

        }
    }

    // Helper method to set motor power
    private void setMotorPower(double fl, double fr, double bl, double br) {
        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }
}
