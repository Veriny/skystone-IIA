package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
    private Servo capServo;
    private int v4bMotorRestPos = 25;
    private int v4bMotorLiftPos = 325;
    private int v4bMotorLastResort = -850;
    private int v4bMotorDumpPos = -525;
    private int liftMotorRestPos = 50;
    private int liftMotorDropPos = 300;
    private int liftMotorDumpPos = 1200;
    private DcMotorControllerEx v4bControllerEx;

    public Lift(DcMotor liftMotor, DcMotor v4bMotor, Servo clawServo, Servo capServo, boolean isAuto) {
//        v4bControllerEx = (DcMotorControllerEx)v4bMotor.getController();
//        PIDFCoefficients pidNew = new PIDFCoefficients(0, 0.0, 0.0, 0.004);
//
//        v4bControllerEx.setPIDFCoefficients(0, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        this.capServo = capServo;

        if(isAuto) {
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
        else {
            this.liftMotor = liftMotor;
            this.v4bMotor = v4bMotor;
            this.clawServo = clawServo;

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setPower(0.0);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            v4bMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            v4bMotor.setPower(0.0);
            v4bMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void controls(Gamepad gp) {

        if(Math.abs(gp.right_stick_y) > 0.05) {
            //slows down the falling down of lift during teleOP
            if(gp.right_stick_y < 0) {
                if(gp.left_trigger != 0) {
                    runLiftMotor(-gp.right_stick_y * 2 / 3);
                }
                else{
                    runLiftMotor(-gp.right_stick_y);
                }
            }
            else {
                //checks if lift goes below 0
                if(liftMotor.getCurrentPosition() <= 25) {
                    holdLiftMotor();
                }
                else {
                    if(gp.left_trigger != 0) {
                        runLiftMotor(-gp.right_stick_y / 6);
                    }
                    else{
                        runLiftMotor(-gp.right_stick_y / 3);
                    }
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

        if(gp.dpad_left) {
            holdCap();
        }
        else if(gp.dpad_right) {
            dropCap();
        }


        if(gp.a) {
            restV4BMotor();
        }
        else if(gp.b) {
            liftV4BMotor();
        }
        else if(gp.y) {
            dumpV4BMotor();
        }
        else if(gp.x) {
            lastResortV4BMotor();
        }
    }


    public synchronized void holdLiftMotor() {
        liftMotor.setPower(0.08);
    }

    public synchronized void runLiftMotor(double power) {
        liftMotor.setPower(power);
    }

    public synchronized void hold() {
        clawServo.setPosition(0.5);
    }

    public synchronized void release() {
        clawServo.setPosition(0.2);
    }

    public synchronized void holdCap() {
        capServo.setPosition(0.1);
    }

    public synchronized void dropCap() {
        capServo.setPosition(0.5);
    }

    public synchronized void restV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorRestPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.5);
    }

    public synchronized void liftV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorLiftPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.7);
    }

    public synchronized void dumpV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorDumpPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.85);
    }

    public synchronized void lastResortV4BMotor() {
        v4bMotor.setTargetPosition(v4bMotorLastResort);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.95);
    }





    public void restLiftMotorNoSync() {
        liftMotor.setTargetPosition(liftMotorRestPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.25);
    }

    public void testLiftMotorNoSync() {
        liftMotor.setTargetPosition(100);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.5);
    }

    public void dropLiftMotorNoSync() {
        liftMotor.setTargetPosition(liftMotorDropPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.5);
    }

    public void dumpLiftMotorNoSync() {
        liftMotor.setTargetPosition(liftMotorDumpPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.75);
    }

    public void restV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorRestPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.5);
    }

    public void liftV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorLiftPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.7);
    }

    public void dumpV4BMotorNoSync() {
        v4bMotor.setTargetPosition(v4bMotorDumpPos);
        v4bMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bMotor.setPower(0.6);
    }


    public void holdNoSync() {
        clawServo.setPosition(0.5);
    }

    public void releaseNoSync() {
        clawServo.setPosition(0.2);
    }

//    public PIDFCoefficients liftInfo() {
//        return v4bControllerEx.getPIDFCoefficients(0, DcMotor.RunMode.RUN_USING_ENCODER);
//    }
}
