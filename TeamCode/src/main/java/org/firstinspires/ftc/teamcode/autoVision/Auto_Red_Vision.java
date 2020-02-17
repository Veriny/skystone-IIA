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

@Autonomous(name="RED RED RED(VISION)_Collect_Deposit_Foundation_Park", group = "test")
public class Auto_Red_Vision extends LinearOpMode {
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
        vision = new SkystoneContour();

        waitForStart();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);
        lift.releaseNoSync();
        robot.update();
//        robot.strafe(24, 0.4); //changed
        robot.drive(29, 0.6);   //added
        robot.residentSleeper(500);
        robot.strafe(6, 0.5);
        robot.turn(-110, 0.5);   //added
        robot.drive(9, 0.5);
        robot.residentSleeper(250);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        robot.residentSleeper(1000);
        vision.setSkystoneFalse();

        robot.update();
        lift.liftV4BMotorNoSync();

        boolean foundSkystone = false;
        int count = 0;


        if (!vision.getStoneCentered()) {
            robot.drive(8.5, 0.35);
            count += 8.5;
            robot.residentSleeper(750);
        }
        else {
            foundSkystone = true;
        }

        if(vision.getStoneCentered()) {
            foundSkystone = true;
        }

        if(foundSkystone) {
            robot.drive(-9, 0.5);
        }

        telemetry.addData("contourCount", vision.getContoursFound());
        telemetry.addData("Skystone found", vision.getStoneCentered());
        telemetry.addData("Width", vision.getWidth());
        telemetry.addData("Height", vision.getHeight());
        telemetry.addData("SkystoneXPos", vision.getSkystoneCameraXPos());
        telemetry.addData("SkystoneYPos", vision.getSkystoneCameraYPos());
        telemetry.addData("Skystone Area", vision.getArea());
        telemetry.update();


        //here's where you add vision

//        boolean foundSkystone = false;
//        int count = 0;
//        for(int i = 0; i < 2; i++) {
//            if(i == 0) {
//                robot.drive(9.5, 0.35);
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//                robot.residentSleeper(1000);
//            }
//            else {
//                if (!vision.getStoneCentered()) {
//                    telemetry.addData("contourCount", vision.getContoursFound());
//                    telemetry.addData("Skystone found", vision.getStoneCentered());
//                    telemetry.addData("Width", vision.getWidth());
//                    telemetry.addData("Height", vision.getHeight());
//                    telemetry.addData("SkystoneXPos", vision.getSkystoneCameraXPos());
//                    telemetry.addData("SkystoneYPos", vision.getSkystoneCameraYPos());
//                    telemetry.update();
//                    robot.drive(9.5, 0.35);
//
//                    //                vision.setSkystoneFalse();
//                    robot.residentSleeper(500);
//                    telemetry.addLine("Entered loop");
//                    count += 8;
//                } else {
//                    foundSkystone = true;
//                    telemetry.addData("Found Skystone", 0);
//                    break;
//                }
//            }
//        }

        telemetry.addData("contourCount", vision.getContoursFound());
        telemetry.addData("Skystone found", vision.getStoneCentered());
        telemetry.addData("Width", vision.getWidth());
        telemetry.addData("Height", vision.getHeight());
        telemetry.addData("SkystoneXPos", vision.getSkystoneCameraXPos());
        telemetry.addData("SkystoneYPos", vision.getSkystoneCameraYPos());
        telemetry.update();
        robot.turn(80, 0.4);
        intake.succNoSync(0.69420);
        robot.drive(24, 0.4);  //changed
        intake.noSuccNoSync();
        robot.residentSleeper(200);
        // robot.strafe(-6, 0.5);

        lift.restV4BMotorNoSync();
        robot.residentSleeper(250);
        robot.drive(-25, 0.8);
        lift.liftV4BMotorNoSync();
        robot.residentSleeper(100);
        lift.restV4BMotorNoSync();
        robot.residentSleeper(400);
        lift.holdNoSync();
        //robot.strafe(8, 0.4);

        phoneCam.stopStreaming();
        robot.turn(-80, 0.4); //changed
        robot.drive(-98  - count, 0.9);  //changed
        robot.turn(-110, 0.5);

        lift.liftV4BMotorNoSync();
        robot.residentSleeper(400);
        lift.dumpLiftMotorNoSync();
        robot.residentSleeper(800);    //changed
        lift.dumpV4BMotorNoSync();
        robot.residentSleeper(250);    //changed
        robot.drive(-14, 0.5); //changed
        lift.dropLiftMotorNoSync();
        robot.residentSleeper(1000);
        lift.releaseNoSync();
        lift.dumpLiftMotorNoSync();
        foundationClaw.pushNoSync();
        robot.residentSleeper(500);
        lift.restV4BMotorNoSync();
        robot.residentSleeper(500);
        lift.restLiftMotorNoSync();
        robot.residentSleeper(500);
        robot.arcTurn(150, 11, 0.75, true);   //changed
        foundationClaw.restNoSync();

        robot.drive(-20, 0.8);  //changed
        robot.strafe(24, 0.7);  //changed
        robot.drive(28, 0.8); //changed



    }
}
