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
        if (gp.a) succ(0.69420);
        else if (gp.b) dontsucc(0.69420);
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
