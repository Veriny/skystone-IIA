package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotor liftMotor;
    //this one is the motor that controls the 4-bar
    private DcMotor v4bMotor;
    private Servo clawServo;
    private int v4bMotorRestPos = 0;
    private int v4bMotorLiftPos = 150;
    private int v4bMotorDumpPos = -250;
    private int liftMotorRestPos = 100;
    private int liftMotorDumpPos = 800;

    public Lift(DcMotor liftMotor, DcMotor v4bMotor, Servo clawServo, boolean isAuto) {
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)v4bMotor.getController();
        PIDFCoefficients pidNew = new PIDFCoefficients(0.1, 0.05, 0, 0.1);

        int motorIndex = ((DcMotorEx)v4bMotor).getPortNumber();
        motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        if(isAuto) {
            this.liftMotor = liftMotor;
            this.v4bMotor = v4bMotor;
            this.clawServo = clawServo;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v4bMotor.setPower(0.0);
            v4bMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            this.liftMotor = liftMotor;
            this.v4bMotor = v4bMotor;
            this.clawServo = clawServo;

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            v4bMotor.setPower(0.0);
            v4bMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void controls(Gamepad gp) {

        runV4BMotor(-gp.right_stick_y);

        if(Math.abs(gp.left_stick_y) > 0.05) {
            //slows down the falling down of lift during teleOP
            if(gp.left_stick_y < 0) {
                runLiftMotor(-gp.left_stick_y);
            }
            else {
                //checks if lift goes below 0
                if(liftMotor.getCurrentPosition() <= 25) {
                    holdLiftMotor();
                }
                else {
                    runLiftMotor(-gp.left_stick_y / 4);
                }
            }
        }
        else {
            holdLiftMotor();
        }

        if(gp.dpad_up) {
            release();
        }
        else if(gp.dpad_down) {
            hold();
        }


        if(gp.a) {
            restV4BMotorNoSync();
        }
        else if(gp.b) {
            liftV4BMotorNoSync();
        }
        else if(gp.y) {
            dumpV4BMotorNoSync();
        }
    }


    public synchronized void holdLiftMotor() {
        liftMotor.setPower(0.08);
    }

    public synchronized void runLiftMotor(double power) {
        liftMotor.setPower(power * 0.75);
    }

    public synchronized void runV4BMotor(double power) {
        v4bMotor.setPower(power / 2.0);
    }


    public synchronized void hold() {
        clawServo.setPosition(0.4);
    }

    public synchronized void release() {
        clawServo.setPosition(0.2);
    }









    public void restLiftMotorNoSync() {
        liftMotor.setTargetPosition(liftMotorRestPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.25);
    }

    public void dumpLiftMotorNoSync() {
        liftMotor.setTargetPosition(liftMotorDumpPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.6);
    }

    public void restV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorRestPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }

    public void liftV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorLiftPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }

    public void dumpV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorDumpPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.25);
    }


    public void holdNoSync() {
        clawServo.setPosition(0.5);
    }

    public void releaseNoSync() {
        clawServo.setPosition(0.0);
    }
}
