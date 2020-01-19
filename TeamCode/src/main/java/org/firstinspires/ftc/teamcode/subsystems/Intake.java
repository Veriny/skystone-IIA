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
        if (gp.left_bumper) succ(0.69420);
        else if (gp.right_bumper) dontsucc(0.69420);
        else noSucc();
    }

    public synchronized void succ(double power) {
        left.setPower(power);
        right.setPower(-power);
    }

    public synchronized void dontsucc(double power) {
        left.setPower(-power);
        right.setPower(power);
    }

    public synchronized void noSucc() {
        left.setPower(0.0);
        right.setPower(0.0);
    }
}
