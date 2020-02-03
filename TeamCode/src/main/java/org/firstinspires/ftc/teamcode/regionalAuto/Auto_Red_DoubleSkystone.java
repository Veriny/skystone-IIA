package org.firstinspires.ftc.teamcode.regionalAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneContour;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="RED RED RED(VISION)_DoubleSkystone", group = "test")
public class Auto_Red_DoubleSkystone extends LinearOpMode {
    public Drivetrain robot;
    public Intake intake;
    public Lift lift;
    public FoundationClaw foundationClaw;
    public SkystoneContour vision;
    public OpenCvCamera phoneCam;
    public ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Drivetrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry);
        intake = new Intake(hardwareMap.dcMotor.get("leftIntake"), hardwareMap.dcMotor.get("rightIntake"));
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"), true);
        foundationClaw = new FoundationClaw(hardwareMap.servo.get("leftFoundationServo"), hardwareMap.servo.get("rightFoundationServo"));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        vision = new SkystoneContour();
        time = new ElapsedTime();


        //80 = distance from foundation to center skystone?
        waitForStart();
        time.reset();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);
        lift.releaseNoSync();
        robot.update();
        robot.strafe(24, 0.7);

        robot.update();

        //here's where you add vision
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        robot.residentSleeper(500);
        int distanceFromCenterSkystone = 0;
        //if skystonePos = left {
            robot.drive(8, 0.7);
            distanceFromCenterSkystone = -8;
        // }

        //else if skystonePos = center {

        // }

        boolean rightStoneTaken = false;
        //else {
            robot.drive(-8, 0.7);
            distanceFromCenterSkystone = 8;
            rightStoneTaken = true;
        // }

        robot.strafe(6, 0.5);
        //pick up block with grabber
        robot.strafe(-6, 0.5);

        robot.drive(-80 + distanceFromCenterSkystone, 0.9);
        robot.strafe(6, 0.6);
        //drop block



        robot.strafe(-6, 0.6);
        robot.drive(80 - distanceFromCenterSkystone + 24, 0.9);
        robot.strafe(6, 0.6);
        //pick up block
        robot.strafe(-6, 0.6);
        robot.drive(-80 + distanceFromCenterSkystone - 24, 0.9);
        robot.strafe(6, 0.6);
        //drop block
        robot.strafe(-6, 0.6);



        if(time.milliseconds() <= 15 * 1000) {
            if(rightStoneTaken) {
                robot.drive(80, 0.9);
                robot.strafe(6, 0.6);
                //pick up block
                robot.strafe(-6, 0.6);
                robot.drive(-80, 0.9);
                robot.strafe(6, 0.6);
                //drop block
                robot.strafe(-6, 0.6);
            }
            else {
                robot.drive(80 - 8, 0.9);
                robot.strafe(6, 0.6);
                //pick up block
                robot.strafe(-6, 0.6);
                robot.drive(-80 + 8, 0.9);
                robot.strafe(6, 0.6);
                //drop block
                robot.strafe(-6, 0.6);
            }
        }

        //fix turning
        robot.turn(-110, 0.6);
        robot.drive(-8, 0.8);
        foundationClaw.pushNoSync();
        robot.residentSleeper(250);
        robot.arcTurn(150, 10, 0.8, true);
        foundationClaw.restNoSync();
        robot.drive(-16, 0.8);
        robot.strafe(16, 0.7);
        robot.drive(20, 0.9);

    }
}
