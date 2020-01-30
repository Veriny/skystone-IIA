package org.firstinspires.ftc.teamcode.autoVision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneContour;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="BLUE BLUE BLUE(VISION)_Collect_Deposit_Foundation_Park", group = "test")
public class Auto_Blue_Vision extends LinearOpMode {
    public Drivetrain robot;
    public Intake intake;
    public Lift lift;
    public FoundationClaw foundationClaw;
    public SkystoneContour vision;
    public OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Drivetrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry);
        intake = new Intake(hardwareMap.dcMotor.get("leftIntake"), hardwareMap.dcMotor.get("rightIntake"));
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"), true);
        foundationClaw = new FoundationClaw(hardwareMap.servo.get("leftFoundationServo"), hardwareMap.servo.get("rightFoundationServo"));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        waitForStart();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        lift.releaseNoSync();
        robot.update();
//        robot.strafe(24, 0.4); //changed
        robot.drive(24, 0.5);   //added
        robot.strafe(-6, 0.4);
        robot.turn(45, 0.5);   //added

        robot.update();
        lift.liftV4BMotorNoSync();

        //here's where you add vision
        intake.succNoSync(0.69420 * 1.1);
        robot.drive(27, 0.3);  //changed
        intake.noSuccNoSync();
        robot.residentSleeper(200);
        intake.dontsuccNoSync(0.5);
        robot.residentSleeper(500);
        intake.noSuccNoSync();
        robot.drive(-2, 0.4);
        robot.drive(4, 0.9);
        robot.strafe(4, 0.4);

        lift.restV4BMotorNoSync();
        robot.residentSleeper(250);
        lift.holdNoSync();
        robot.drive(-35, 0.8);
        robot.strafe(-8, 0.4);
        lift.liftV4BMotorNoSync();
        robot.residentSleeper(200);
        lift.restV4BMotorNoSync();

        phoneCam.stopStreaming();
        robot.turn(45, 0.4); //changed
        robot.drive(-57, 0.75);  //changed
        robot.turn(90, 0.5);

        lift.dumpLiftMotorNoSync();
        robot.residentSleeper(750);    //changed
        lift.dumpV4BMotorNoSync();
        robot.residentSleeper(500);    //changed
        robot.drive(-13, 0.4); //changed
        lift.releaseNoSync();
        foundationClaw.pushNoSync();
        robot.residentSleeper(500);
        lift.restV4BMotorNoSync();
        robot.residentSleeper(500);
        lift.restLiftMotorNoSync();
        robot.residentSleeper(500);
        robot.arcTurn(135, 12, 0.275, false);   //changed
        robot.residentSleeper(250); //changed
        foundationClaw.restNoSync();

        robot.drive(-22, 0.5);  //changed
        robot.strafe(-19, 0.6);  //changed
        robot.drive(30, 0.5); //changed



    }
}
