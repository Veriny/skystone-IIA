package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SkystoneContour;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "pepega", group = "pepehands")
public class VisionTestOP extends LinearOpMode {
    public Drivetrain robot;
    public SkystoneContour vision;
    public OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Drivetrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        vision = new SkystoneContour();
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(vision);
        waitForStart();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        for(int i = 0; i < 5; i++) {
            telemetry.addLine("X Position: " + vision.getContourXPosTest());
            robot.residentSleeper(1000);
        }
        if(robot.skystoneIsCentered()) {
            telemetry.addLine("Camera working");
        }
    }
}
