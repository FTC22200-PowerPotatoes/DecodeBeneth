package org.firstinspires.ftc.teamcode.unusedAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto_right")
//@Disabled
public class auto_right extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {
        // Initialize the motors
        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {

            // Step 2: Move up
            setMotorPower(0.5, 0.5, 0.5, 0.5);
            sleep(300); // Move up for 0.3 seconds

            setMotorPower(0.5,-0.5,-0.5,0.5);
            sleep(2800); // Strafe right for 2.8 seconds

            setMotorPower(-0.5,-0.5,-0.5,-0.5);
            sleep(400); // Move down for 0.4 seconds

            // Stop all motors
            setMotorPower(0, 0, 0, 0);
        }
    }

    private void setMotorPower(double fl, double fr, double bl, double br) {
        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }
}
