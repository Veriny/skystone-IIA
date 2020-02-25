package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class Gyrotrain extends Drivetrain{
    private static final double HEADING_THRESHOLD = 1;
    private static final double P_TURN_COEFF = .1;
    private static final double P_DRIVE_COEFF = .15;
    private BNO055IMU imu;
    private static Orientation prevAngle;
    private static double globalAngle = 0;
    private static final double ANGLE_CORRECTION = .3;
    public Gyrotrain(DcMotor tl, DcMotor bl, DcMotor tr, DcMotor br, Boolean isAuto, Telemetry t, HardwareMap h) throws InterruptedException{
        super(tl, bl, tr, br, isAuto, t, h);
        this.imu = h.get(BNO055IMU.class, "imu"); // Internally connected to I2C port 0 and configured to address 0x28
        prevAngle = new Orientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            sleep(50);
        }

    }

    private void resetAngle() { //Basically resets the z angle back to zero lmao
        prevAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double checkDirection() {
        double gain = .1;
        double correction = 0;
        if (getAngle() == 0)  {
            correction = 0;
        }
        else {
            correction = -.1;
        }
        correction = correction * gain;
        return correction;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double dAngle = angles.firstAngle - prevAngle.firstAngle;

        //here, we make use of euler angles — use victor's explanation to find out what this is lmao
        if (dAngle < -180) {
           dAngle += 180;
        }
        else if (dAngle > 180) {
            dAngle -= 180;
        }
        globalAngle += dAngle;
        prevAngle = angles;
        telemetry.addData("Global angle: ",  globalAngle);
        telemetry.update();
        return globalAngle;
    }

    @Override
    public void drive(double power, double position) {
        double pos = (calculateTicks(position));
        motorDrive(topRight, pos, power);
        motorDrive(topLeft, pos, power);
        motorDrive(bottomRight, pos, power);
        motorDrive(bottomLeft, pos, power);
        resetAngle();

        do {
            if (getAngle() >= .5) {
                topLeft.setPower(power - ANGLE_CORRECTION);
                bottomLeft.setPower(power - ANGLE_CORRECTION);
            }
            else if (getAngle() <= .5) {
                bottomRight.setPower(power - ANGLE_CORRECTION);
                topRight.setPower(power - ANGLE_CORRECTION);
            }
            else {
                bottomRight.setPower(power);
                topRight.setPower(power);
                topLeft.setPower(power);
                bottomLeft.setPower(power);
            }
        } while (topRight.isBusy() || topLeft.isBusy() || bottomRight.isBusy() || bottomLeft.isBusy());
        stop();
        resetEncoders();
    }

    public void motorDrive(DcMotor motor, double ticks) {
        motor.setTargetPosition((int) ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnByIMU(double degrees, double power) throws InterruptedException {
        resetAngle();
        motorRunWithoutEncoder();
        sleep(100);
        if(degrees > 0) {
            turnWithoutEncoder(power);
        }
        else {
            turnNoEncoder(-power);
        }
        while(Math.abs(getAngle()) < Math.abs(degrees)) {

        }
        stop();
        motorRunByEncoder();
        sleep(250);
        resetAngle();
    }

    private void turnWithoutEncoder(double power) {
        topLeft.setPower(power);
        topRight.setPower(-power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
    }



}

