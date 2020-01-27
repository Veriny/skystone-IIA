package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
    private DcMotor left;
    private DcMotor right;
    public Intake(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;
    }

    public void controls(Gamepad gp) {
        if (gp.left_bumper) dontsucc(0.69420);
        else if (gp.right_bumper) succ(0.69420);
        else if(gp.right_trigger != 0) {

        }
        else noSucc();
    }

    public synchronized void succ(double power) {
        left.setPower(power * 1.03);
        right.setPower(-power);
    }

    public synchronized void dontsucc(double power) {
        left.setPower(-power * 1.03);
        right.setPower(power);
    }

    public synchronized void noSucc() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    public synchronized void slowSucc() {
        left.setPower(0.45);
        right.setPower(-0.45);
    }




    public void succNoSync(double power) {
        left.setPower(power * 1.03);
        right.setPower(-power);
    }

    public void dontsuccNoSync(double power) {
        left.setPower(-power * 1.03);
        right.setPower(power);
    }

    public void noSuccNoSync() {
        left.setPower(0.0);
        right.setPower(0.0);
    }
}
