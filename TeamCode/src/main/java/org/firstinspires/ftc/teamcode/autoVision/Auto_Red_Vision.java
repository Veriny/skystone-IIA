package org.firstinspires.ftc.teamcode.autoVision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Gyrotrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneContour;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="RED RED RED(VISION)_Collect_Deposit_Foundation_Park", group = "test")
public class Auto_Red_Vision extends LinearOpMode {
    public Gyrotrain robot;
    public Intake intake;
    public Lift lift;
    public FoundationClaw foundationClaw;
    public SkystoneContour vision;
    public OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Gyrotrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry, hardwareMap);
        intake = new Intake(hardwareMap.dcMotor.get("leftIntake"), hardwareMap.dcMotor.get("rightIntake"));
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"), hardwareMap.servo.get("capServo"), true, telemetry);
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
        robot.drive(29, 0.7, 3);   //added
        robot.strafe(6, 0.5);
        robot.turn(-90, 0.8);   //added
        robot.drive(8.5, 0.35);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        robot.residentSleeper(750);
        vision.setSkystoneFalse();

        robot.update();
        lift.liftV4BMotorNoSync();

        boolean foundSkystone = false;
        int count = 0;


        if (!vision.getStoneCentered()) {
            robot.drive(8.5, 0.35);
            count += 8.5;
            robot.residentSleeper(1000);
        }
        else {
            foundSkystone = true;
        }

        if(vision.getStoneCentered()) {
            foundSkystone = true;
        }

        if(foundSkystone) {
            robot.drive(-9, 0.5, 3);
            count -= 9;
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
//                    robot.drive(9.5, 0.35);fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
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
        robot.turn(62.5, 0.8);
        intake.succNoSync(0.69420);
        robot.drive(24, 0.4, 3);  //changed
        intake.noSuccNoSync();
        robot.residentSleeper(200);
        // robot.strafe(-6, 0.5);

        lift.restV4BMotorNoSync();
        robot.residentSleeper(250);
        robot.drive(-25, 1.0);
        lift.liftV4BMotorNoSync();
        robot.residentSleeper(100);
        lift.restV4BMotorNoSync();
        robot.residentSleeper(300);
        lift.holdNoSync();
        //robot.strafe(8, 0.4);

        phoneCam.stopStreaming();
        robot.turn(-62.5, 0.8); //changed
        robot.drive(-86  - count, 1.0, 3);  //changed
//        robot.setNewAngle(-90);
        robot.residentSleeper(250);
        robot.turn(0, 0.5);
        robot.turnByEncoder(-90, 0.6);
//        robot.turn(-90, 0.8);
        lift.dumpLiftMotorNoSync();
        robot.drive(-14, 0.7); //changed
        lift.dumpV4BMotorNoSync();
        robot.residentSleeper(500);
        lift.dropLiftMotorNoSync();
        robot.residentSleeper(500);
        lift.releaseNoSync();
        lift.dumpLiftMotorNoSync();
        foundationClaw.pushNoSync();
        lift.restV4BMotorNoSync();
        robot.residentSleeper(500);
        lift.restLiftMotorNoSync();
        robot.residentSleeper(500);
        robot.arcTurn(155, 11, 1.0, true);   //changed
        foundationClaw.restNoSync();
        robot.drive(-20, 1.0);  //changed
        robot.turn(0, 1.0);
        robot.strafe(24, 1.0);  //changed
        robot.residentSleeper(250);
        robot.drive(28, 1.0); //changed



    }
}
