package org.firstinspires.ftc.teamcode.ExperimentationAutonomous;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.intakeOuttake;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

// Limelight

@Autonomous(name="RRfrontRed", group="Decode")
public class frontRed extends LinearOpMode {
    public Limelight3A limelight;
    public IMU imu;

    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(new Vector2d(-49.5, 49.5), Math.toRadians(130));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        intakeOuttake take = new intakeOuttake(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        double distance;

        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        limelight.start();

        if (isStopRequested()) return;

        LLResult llResult = limelight.getLatestResult();
        boolean isValid = llResult != null && llResult.isValid();


        if (isValid) {
            distance = getDistanceFromTag(llResult.getTy());
        } else {
            distance = 280.0;
        }


        /* AUTO AIM
        if (gamepad1.right_trigger > 0.0 && isValid) {
            double tx = llResult.getTx();
            double kP = 0.02;
            double turnPower = kP * (tx + 3);
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
            if (Math.abs(tx) < 1.0) turnPower = 0;

            fron.setPower(-turnPower);
            bL_Motor += -turnPower;
            fR_Motor += turnPower;
            bR_Motor += turnPower;
        }
        */

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                // Preload balls
                .afterTime(0, take.revLauncher(1300))
                .strafeToSplineHeading(new Vector2d(-24.5, 24.5), Math.toRadians(125))
                .afterTime(0, take.turnToTag(true, false))
                .afterTime(0, take.shootBalls())
                .waitSeconds(1.8)
                .afterTime(0, take.turnToTag(false, false))
                .afterTime(0, take.stopBalls())
                // First balls
                .strafeToLinearHeading(new Vector2d(-24.5, 26.5), Math.toRadians(270.0))
                .afterTime(0, take.intakeBalls())
                .afterTime(0.4, take.revLauncher(1300))
                .strafeTo(new Vector2d(-24.5, 38))
                .afterTime(0, take.stopBalls())
                .afterTime(0, take.revLauncher(1300))
                .strafeToSplineHeading(new Vector2d(-24.5, 24.5), Math.toRadians(125))
                .afterTime(0, take.turnToTag(true, false))
                .afterTime(0, take.shootBalls())
                .waitSeconds(1.8)
                .afterTime(0, take.turnToTag(false, false))
                .afterTime(0, take.stopBalls())
                // Second balls
                .strafeToLinearHeading(new Vector2d(-9.5, 26.5), Math.toRadians(270.0))
                .afterTime(0, take.intakeBalls())
                .afterTime(0.4, take.revLauncher(1300))
                .strafeTo(new Vector2d(-9.5, 38))
                .afterTime(0, take.stopBalls())
                .afterTime(0, take.revLauncher(1300))
                .strafeToSplineHeading(new Vector2d(-24.5, 24.5), Math.toRadians(125))
                .afterTime(0, take.turnToTag(true, false))
                .afterTime(0, take.shootBalls())
                .waitSeconds(1.8)
                .afterTime(0, take.turnToTag(false, false))
                .afterTime(0, take.stopBalls())
                // Third balls
                .strafeToLinearHeading(new Vector2d(5.5, 26.5), Math.toRadians(270.0))
                .afterTime(0, take.intakeBalls())
                .afterTime(0.4, take.revLauncher(1300))
                .strafeTo(new Vector2d(5.5, 38))
                .afterTime(0, take.stopBalls())
                .afterTime(0, take.revLauncher(1300))
                .strafeToSplineHeading(new Vector2d(-24.5, 24.5), Math.toRadians(125))
                .afterTime(0, take.turnToTag(true, false))
                .afterTime(0, take.shootBalls())
                .waitSeconds(1.8)
                .afterTime(0, take.turnToTag(false, false))
                .afterTime(0, take.stopBalls())
                .strafeTo(new Vector2d(-5.5, 30))
                .build()
        );



    }
    private double calculateRPM(double distance) {
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
    private double getDistanceFromTag(double ty) {
        double cameraHeight = 33.7;
        double tagHeight = 76;
        double cameraAngle = Math.toRadians(3.1);

        double angleToTag = cameraAngle + Math.toRadians(ty);
        return (tagHeight - cameraHeight) / Math.tan(angleToTag); // meters
    }


}