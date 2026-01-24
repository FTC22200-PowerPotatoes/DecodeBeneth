package org.firstinspires.ftc.teamcode.OldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto_right")
//@Disabled
public class auto_right extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {
        // Initialize the motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

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
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
