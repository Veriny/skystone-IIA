package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Gyrotrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name="IMU Turn Test", group = "test")
public class IMUTest extends LinearOpMode {
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
        robot.turn(60, 0.5);
        robot.turn(90, 0.5);
        robot.drive(4, 0.5);
        robot.turn(-150, 0.8);
        robot.drive(2, 0.5);
        robot.turn(30, 1.0);
    }
}
