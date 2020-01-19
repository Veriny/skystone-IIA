package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotor liftMotor;
    //this one is the motor that controls the 4-bar
    private DcMotor v4bMotor;
    private Servo clawServo;
    private int v4bMotorRestPos = 0;
    private int v4bMotorLiftPos = 400;
    private int v4bMotorDumpPos = 800;
    private int liftMotorRestPos = 0;
    private int liftMotorDumpPos = -800;

    public Lift(DcMotor liftMotor, DcMotor v4bMotor, Servo clawServo) {
        this.liftMotor = liftMotor;
        this.v4bMotor = v4bMotor;
        this.clawServo = clawServo;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(0.0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4bMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        v4bMotor.setPower(0.0);
        v4bMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void controls(Gamepad gp) {
        runLiftMotor(gp.left_stick_y);
        runV4BMotor(gp.right_stick_y);
        if(gp.b) {
            dumpLiftMotor();
        }
        if(gp.a) {
            restV4BMotor();
        }
        if(gp.y) {
            release();
        }
        else if(gp.x) {
            hold();
        }
    }


    public synchronized void runLiftMotor(double power) {
        liftMotor.setPower(power);
    }

    public synchronized void restLiftMotor() {
        liftMotor.setTargetPosition(liftMotorDumpPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.25);
    }

    public synchronized void dumpLiftMotor() {
        liftMotor.setTargetPosition(liftMotorDumpPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.5);
    }

    public synchronized void runV4BMotor(double power) {
        v4bMotor.setPower(power);
    }

    public synchronized void restV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorRestPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }

    public synchronized void liftV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorLiftPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }

    public synchronized void dumpV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorDumpPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }


    public synchronized void hold() {
        clawServo.setPosition(0.5);
    }

    public synchronized void release() {
        clawServo.setPosition(0.0);
    }
}
