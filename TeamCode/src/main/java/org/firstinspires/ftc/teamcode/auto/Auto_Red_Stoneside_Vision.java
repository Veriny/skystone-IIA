package org.firstinspires.ftc.teamcode.auto;

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

@Autonomous(name="Red(Stoneside) Vision", group = "test")
public class Auto_Red_Stoneside_Vision extends LinearOpMode {
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
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"));
        foundationClaw = new FoundationClaw(hardwareMap.servo.get("leftFoundationServo"), hardwareMap.servo.get("rightFoundationServo"));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        waitForStart();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        lift.liftV4BMotor();
        lift.release();
        robot.strafe(18, 0.5);
        boolean hasSkystone = false;
        while(!robot.skystoneIsCentered() && robot.inchesMoved() <= 18) {
            robot.driveNoDist(0.15);

        }
        double moveCount = robot.inchesMoved();
        robot.resetEncoders();
        robot.drive(-4, 0.5);
        robot.turn(45, 0.5);
        intake.succ(0.69420 * 1.1);
        robot.drive(26, 0.2);
        intake.noSucc();
        lift.restV4BMotor();
        lift.hold();
        robot.drive(-16, 0.6);
        hasSkystone = true;


        if(!hasSkystone) {
            robot.drive(-4, 0.5);
            robot.turn(45, 0.5);
            intake.succ(0.69420 * 1.1);
            robot.drive(26, 0.2);
            intake.noSucc();
            lift.restV4BMotor();
            lift.hold();
            robot.drive(-25, 0.6);
        }

        phoneCam.stopStreaming();
        robot.turn(135, 0.4);
        robot.drive(moveCount, 0.5);
        robot.drive(78, 0.6);
        robot.turn(90, 0.5);

        lift.dumpLiftMotor();
        robot.residentSleeper(1500);
        lift.dumpV4BMotor();
        robot.residentSleeper(1000);
        robot.drive(-10, 0.3);
        lift.release();
        foundationClaw.push();
        robot.residentSleeper(500);
        lift.restV4BMotor();
        robot.residentSleeper(1000);
        lift.restLiftMotor();

        robot.drive(28, 0.4);
        foundationClaw.rest();
        robot.drive(-1, 0.5);
        robot.strafe(24, 0.6);
        robot.drive(-20, 0.6);
        robot.strafe(20, 0.5);



    }
}
