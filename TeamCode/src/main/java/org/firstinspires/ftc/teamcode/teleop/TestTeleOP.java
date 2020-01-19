package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.FoundationClaw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


@TeleOp(name = "TestOPMode", group = "jerW" )
public class TestTeleOP extends OpMode {
    //TODO: Create instance variable for drivetrain
    public Drivetrain drivetrain;
    public Lift lift;
    public Intake intake;
    public FoundationClaw foundationClaw;
    @Override
    public void init() {
        //TODO: Initialize Drivetrain Object
        drivetrain = new Drivetrain(hardwareMap.dcMotor.get("topLeftMotor"), hardwareMap.dcMotor.get("bottomLeftMotor"), hardwareMap.dcMotor.get("topRightMotor"), hardwareMap.dcMotor.get("bottomRightMotor"), false, telemetry);
        intake = new Intake(hardwareMap.dcMotor.get("leftIntake"), hardwareMap.dcMotor.get("rightIntake"));
        lift = new Lift(hardwareMap.dcMotor.get("liftMotor"), hardwareMap.dcMotor.get("v4bMotor"), hardwareMap.servo.get("clawServo"));
    }

    @Override
    public void start() {
        super.start();
        telemetry.addLine("Robot started");
    }

    @Override
    public void loop() {
        //TODO: Controls
        drivetrain.controls(gamepad1);
        intake.controls(gamepad1);
        lift.controls(gamepad2);
    }

    @Override
    public void stop() {
        super.stop();
        telemetry.addLine("Robot stopped");
    }
}
