package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;

public class PIDcontroller {
    private double kP;
    private double kI;
    private double kD;
    private double prevError, integralError, derivativeError, positionError, prevTime, power;

    public PIDcontroller(double kP, double kI, double kD) {
        prevError = 0;
        power = 1;
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }


    public double getPower(int ticks, int currPos, double runTime) {
        positionError = ticks - currPos;
        derivativeError = (positionError - prevError) / (runTime  - prevTime);
        integralError += positionError * (runTime - prevTime);
        prevTime = runTime;
        return Range.clip(kP * positionError + kI * integralError + kD * derivativeError, 1, -1);
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    //not necessary if i reinitialize PIDcontroller every time i want to drive
    public void resetPID() {
        positionError = 0;
        derivativeError = 0;
        integralError = 0;
    }

    public double getPrevError() {
        return prevError;
    }
}

