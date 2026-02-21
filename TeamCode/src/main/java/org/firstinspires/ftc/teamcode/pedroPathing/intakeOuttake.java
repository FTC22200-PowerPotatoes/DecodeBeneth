package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class intakeOuttake {
    double launcher_Velocity1 = 1200.0;
    public DcMotor intakeMotor;
    public DcMotorEx launcher;
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public DcMotor boxMotor;
    public CRServo leftFeeder;
    public CRServo rightFeeder;
    public CRServo topWheel;
    public AnalogInput laserAnalog;
    public Limelight3A limelight;
    public IMU imu;
    private ElapsedTime turningTime = new ElapsedTime();
    double F = 14.8;
    double P = 430.0;
    public double distance;
    public static final double MAX_VOLTS = 3.3;
    public static final double MAX_DISTANCE_MM = 4000.0;



    public intakeOuttake(HardwareMap hardwareMap) {
        laserAnalog = hardwareMap.get(AnalogInput.class, "laserAnalogInput");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        boxMotor = hardwareMap.get(DcMotorEx.class, "boxMotor");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients initialPidf = new PIDFCoefficients(P, 0.0, 0.0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, initialPidf);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);


        // IMU HERE!
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    public class intakeBalls implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            leftFeeder.setPower(-1.0);
            rightFeeder.setPower(1.0);
            intakeMotor.setPower(1.0);

            return false;
        }

    }

    public class stopBalls implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(0.0);
            leftFeeder.setPower(0.0);
            rightFeeder.setPower(0.0);
            boxMotor.setPower(0.0);
            topWheel.setPower(0.0);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setPower(0.0);

            return false;
        }
    }



    public class turnToTag implements Action {
        boolean turn;
        boolean blue;
        public turnToTag(boolean turn, boolean blue) {
            this.turn = turn;
            this.blue = blue;
            turningTime.reset();

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            while (turn && turningTime.seconds() < 1.0) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
                LLResult llResult = limelight.getLatestResult();
                boolean isValid = llResult != null && llResult.isValid();
                double motiffID = 0;
                assert llResult != null;
                for(LLResultTypes.FiducialResult fid :llResult.getFiducialResults()) {
                    motiffID = fid.getFiducialId();
                } if (turn && isValid && ((motiffID == 20 && blue) || (motiffID == 24 && !blue))) {
                        double tx = llResult.getTx();
                        // 'amt' of turn
                        double kP = 0.02;
                        double turnPower = kP * (tx);
                        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
                        if (Math.abs(tx+3) < 1.0)  turn = false; // may neeed to be 2.5 < math.abs(tx) < 3.5 //Example for offset of 3 with 0.5 margins


                        // Rotate robot via above
                        leftFront.setPower(-turnPower);
                        leftBack.setPower(-turnPower);
                        rightFront.setPower(turnPower);
                        rightBack.setPower(turnPower);


                        // Telemetry for data
                } else {
                    /*if (blue) {
                        leftFront.setPower(1.0);
                        leftBack.setPower(1.0);
                        rightFront.setPower(-1.0);
                        rightBack.setPower(-1.0);
                    } else {
                        leftFront.setPower(-1.0);
                        leftBack.setPower(-1.0);
                        rightFront.setPower(1.0);
                        rightBack.setPower(1.0);
                    }
                    */
                }


            }

            return false;
        }
    }


    public class shootBalls implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(1.0);
            leftFeeder.setPower(-1.0);
            rightFeeder.setPower(1.0);
            topWheel.setPower(-1.0);
            boxMotor.setPower(1.0);

            return false;
        }
    }

    public class revLauncher implements Action {
        double launcher_Velocity1;

        public revLauncher(double launcher_Velocity1) {
            this.launcher_Velocity1 = launcher_Velocity1;
        }
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            PIDFCoefficients pidf = new PIDFCoefficients(P, 0.0, 0.0, F);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setVelocity(launcher_Velocity1);


            return false;
        }
    }
    public Action intakeBalls() {
        return new intakeBalls();
    }
    public Action stopBalls() {
        return new stopBalls();
    }
    public Action revLauncher(double launcher_Velocity1) {
        return new revLauncher(launcher_Velocity1);
    }
    public Action shootBalls() {
        return new shootBalls();
    }
    public Action turnToTag(boolean turn, boolean blue) {
        return new turnToTag(turn, blue);
    }

}