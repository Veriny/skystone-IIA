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
        vision = new SkystoneContour();
        vision.setShowContours(true);
        vision.setTelemetry(telemetry);
        phoneCam.setPipeline(vision);
        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        if(vision.skystoneIsCentered()) {
            telemetry.addLine("Camera working");
            telemetry.addLine("X-Pos: " + vision.getContourXPos() + ", Y-Pos: " + vision.getContourYPos());
        }
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("Contour count", vision.getContourFoundCount());
            telemetry.addData("X-Pos: ", vision.getContourXPos());
            telemetry.addData("Y-Pos: ", vision.getContourYPos());
            telemetry.update();
        }
    }
}
