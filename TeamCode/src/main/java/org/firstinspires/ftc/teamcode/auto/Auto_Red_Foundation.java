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

@Autonomous(name="Red Foundation Park", group = "test")
public class Auto_Red_Foundation extends LinearOpMode {
    public Drivetrain robot;
    public FoundationClaw foundationClaw;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Drivetrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry);
        foundationClaw = new FoundationClaw(hardwareMap.servo.get("leftFoundationServo"), hardwareMap.servo.get("rightFoundationServo"));

        waitForStart();
        robot.drive(-1, 0.4);
        robot.strafe(-8, 0.5);
        robot.drive(-31, 0.4);
        foundationClaw.pushNoSync();
        robot.residentSleeper(1000);
        robot.drive(-1, 0.3);
        robot.residentSleeper(500);
        robot.arcTurn(135, 12, 0.275, true);
        foundationClaw.restNoSync();
        robot.residentSleeper(500);
        robot.drive(-22, 0.4);
        robot.strafe(19, 0.4);
        robot.drive(30, 0.4);
    }
}
