package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Gyrotrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name="No Euler Test", group = "test")
public class NoEulerTest extends LinearOpMode {
    public Gyrotrain robot;
    public Intake intake;
    public Lift lift;
    public FoundationClaw foundationClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Gyrotrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), true, telemetry, hardwareMap);
        intake = new Intake(hardwareMap.dcMotor.get("leftIntake"), hardwareMap.dcMotor.get("rightIntake"));
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"), hardwareMap.servo.get("capServo"), true);
        foundationClaw = new FoundationClaw(hardwareMap.servo.get("leftFoundationServo"), hardwareMap.servo.get("rightFoundationServo"));

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Angle", robot.getNonEulerAngle());
            telemetry.update();
        }
    }
}

