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
    private double totalAngle = 0;
    private double xToDoubt;
    private static final double ANGLE_CORRECTION = .3;
    public Gyrotrain(DcMotor tl, DcMotor bl, DcMotor tr, DcMotor br, Boolean isAuto, Telemetry t, HardwareMap h) throws InterruptedException{
        super(tl, bl, tr, br, isAuto, t, h);
        telemetry.addData("Gyro calibrating...", true);
        telemetry.update();
        this.imu = h.get(BNO055IMU.class, "imu"); // Internally connected to I2C port 0 and configured to address 0x28
        prevAngle = new Orientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            sleep(50);
        }
        telemetry.addData("Gyro calibrating...", false);
        telemetry.update();


    }

    public void resetAngle() { //Basically resets the z angle back to zero lmao
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

    public double getNonEulerAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        xToDoubt += angle;
        return xToDoubt;
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
        return globalAngle;
    }


    public void drive(double distance, double power, double threshold) {
        double pos = (distance / (WHEEL_DIAMETER * Math.PI * 2)) * 1440;
        motorDrive(topRight, pos, power);
        motorDrive(topLeft, pos, power);
        motorDrive(bottomRight, pos, power);
        motorDrive(bottomLeft, pos, power);
        do {
            telemetry.addData("Angle:", getAngle());
            telemetry.addData("topRight position", topRight.getTargetPosition() - topRight.getCurrentPosition());
            telemetry.addData("botRight position", bottomRight.getTargetPosition() - bottomRight.getCurrentPosition());
            telemetry.addData("topLeft position", topLeft.getTargetPosition() - topLeft.getCurrentPosition());
            telemetry.addData("botLeft position", bottomLeft.getTargetPosition() - bottomLeft.getCurrentPosition());
            telemetry.addData("topLeft power", topLeft.getPower());
            telemetry.addData("botLeft power", bottomLeft.getPower());
            telemetry.addData("topRight power", topRight.getPower());
            telemetry.addData("botRight power", bottomRight.getPower());

            telemetry.update();
            if (getAngle() <= -threshold) {
                if (distance > 0){
                    topLeft.setPower(power - ANGLE_CORRECTION);
                    bottomLeft.setPower(power - ANGLE_CORRECTION);
                    bottomRight.setPower(power);
                    topRight.setPower(power);
                }
                else {
                    bottomRight.setPower(power - ANGLE_CORRECTION);
                    topRight.setPower(power - ANGLE_CORRECTION);
                    bottomLeft.setPower(power);
                    topLeft.setPower(power);
                }
            }
            else if (getAngle() >= threshold) {
                if (distance > 0){
                    bottomRight.setPower(power - ANGLE_CORRECTION);
                    topRight.setPower(power - ANGLE_CORRECTION);
                    bottomLeft.setPower(power);
                    topLeft.setPower(power);
                }
                else {
                    topLeft.setPower(power - ANGLE_CORRECTION);
                    bottomLeft.setPower(power - ANGLE_CORRECTION);
                    bottomRight.setPower(power);
                    topRight.setPower(power);
                }
            }
            else {
                bottomRight.setPower(power);
                topRight.setPower(power);
                topLeft.setPower(power);
                bottomLeft.setPower(power);
            }
        } while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()));
        stop();
        resetEncoders();
    }

    public void driveByEncoder(double distance, double power) {
        super.drive(distance, power);
    }

//    @Override
//    public void strafe(double distance, double power) {
//        double pos = (distance / (WHEEL_DIAMETER * Math.PI * 2)) * 1440;
//        resetAngle();
//        motorDrive(topRight, -pos, power);
//        motorDrive(topLeft, pos, power);
//        motorDrive(bottomRight, pos, power);
//        motorDrive(bottomLeft, -pos, power);
//        do {
//            telemetry.addData("Angle:", getAngle());
//            telemetry.addData("topRight position", topRight.getTargetPosition() - topRight.getCurrentPosition());
//            telemetry.addData("botRight position", bottomRight.getTargetPosition() - bottomRight.getCurrentPosition());
//            telemetry.addData("topLeft position", topLeft.getTargetPosition() - topLeft.getCurrentPosition());
//            telemetry.addData("botLeft position", bottomLeft.getTargetPosition() - bottomLeft.getCurrentPosition());
//            telemetry.addData("topLeft power", topLeft.getPower());
//            telemetry.addData("botLeft power", bottomLeft.getPower());
//            telemetry.addData("topRight power", topRight.getPower());
//            telemetry.addData("botRight power", bottomRight.getPower());
//
//            telemetry.update();
//            if (90 - getAngle() <= -1) {
//                if (distance < 0) tLbRcorrect(power);
//                else bLtRcorrect(power);
//            }
//            else if (90 - getAngle() >= 1) {
//                if (distance < 0) bLtRcorrect(power);
//                else tLbRcorrect(power);
//            }
//            else {
//                bottomRight.setPower(power);
//                topRight.setPower(power);
//                topLeft.setPower(power);
//                bottomLeft.setPower(power);
//            }
//        } while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
//                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()));
//        stop();
//        resetEncoders();
//    }

    //Correction Methods
    private void tLbRcorrect(double power) {
        bottomLeft.setPower(power - ANGLE_CORRECTION);
        topRight.setPower(power - ANGLE_CORRECTION);
        topLeft.setPower(power);
        bottomRight.setPower(power);
    }

    private void bLtRcorrect(double power)  {
        topLeft.setPower(power - ANGLE_CORRECTION);
        bottomRight.setPower(power - ANGLE_CORRECTION);
        topRight.setPower(power);
        bottomLeft.setPower(power);
    }


    public void motorDrive(DcMotor motor, double ticks) {
        motor.setTargetPosition((int) ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void turn(double degrees, double power) throws InterruptedException {
        double rotations = degrees / 360 / 1.7625;
        double position = calculateTicksRot(rotations * BOT_CIRCUMFERENCE);
        sleep(100);
        motorDrive(bottomLeft, position, power);
        motorDrive(bottomRight, -position, power);
        motorDrive(topLeft, position, power);
        motorDrive(topRight, -position, power);
        while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())) {
            telemetry.addData("Angle:", getAngle());
            telemetry.addData("topRight position", topRight.getTargetPosition() - topRight.getCurrentPosition());
            telemetry.addData("botRight position", bottomRight.getTargetPosition() - bottomRight.getCurrentPosition());
            telemetry.addData("topLeft position", topLeft.getTargetPosition() - topLeft.getCurrentPosition());
            telemetry.addData("botLeft position", bottomLeft.getTargetPosition() - bottomLeft.getCurrentPosition());
            telemetry.addData("topLeft power", topLeft.getPower());
            telemetry.addData("botLeft power", bottomLeft.getPower());
            telemetry.addData("topRight power", topRight.getPower());
            telemetry.addData("botRight power", bottomRight.getPower());

            telemetry.update();
        }
        sleep(250);
        resetEncoders();
        telemetry.addData("Encoder turn complete", degrees);
        double degreeDiff = 0;
        if(degrees >= 0) {
            degreeDiff = degrees - Math.abs(getAngle());
        }
        else {
            degreeDiff = degrees + Math.abs(getAngle());
        }
        double correctedRotations = degreeDiff / 360 / 1.7625;
        double correctedPosition = calculateTicksRot(correctedRotations * BOT_CIRCUMFERENCE);
        telemetry.addData("Corrected degrees", degreeDiff);
        telemetry.addData("Corrected rotations", correctedRotations);
        telemetry.addData("Corrected position", correctedPosition);
        telemetry.update();
        motorDrive(bottomLeft, correctedPosition, power);
        motorDrive(bottomRight, -correctedPosition, power);
        motorDrive(topLeft, correctedPosition, power);
        motorDrive(topRight, -correctedPosition, power);
        while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())) {
            telemetry.addData("Corrected degrees", degreeDiff);
            telemetry.addData("Corrected position", correctedPosition);
            telemetry.addData("Angle:", getAngle());
            telemetry.addData("topRight position", topRight.getTargetPosition() - topRight.getCurrentPosition());
            telemetry.addData("botRight position", bottomRight.getTargetPosition() - bottomRight.getCurrentPosition());
            telemetry.addData("topLeft position", topLeft.getTargetPosition() - topLeft.getCurrentPosition());
            telemetry.addData("botLeft position", bottomLeft.getTargetPosition() - bottomLeft.getCurrentPosition());
            telemetry.addData("topLeft power", topLeft.getPower());
            telemetry.addData("botLeft power", bottomLeft.getPower());
            telemetry.addData("topRight power", topRight.getPower());
            telemetry.addData("botRight power", bottomRight.getPower());

            telemetry.update();
        }
        sleep(250);
    }


    public void turnFromLastReset(double degrees, double power) throws InterruptedException {
        double degreeDiff = degrees + getAngle();
        double correctedRotations = degreeDiff / 360 / 1.7625;
        double correctedPosition = calculateTicksRot(correctedRotations * BOT_CIRCUMFERENCE);
        telemetry.addData("Corrected degrees", degreeDiff);
        telemetry.addData("Corrected rotations", correctedRotations);
        telemetry.addData("Corrected position", correctedPosition);
        telemetry.update();
        motorDrive(bottomLeft, correctedPosition, power);
        motorDrive(bottomRight, -correctedPosition, power);
        motorDrive(topLeft, correctedPosition, power);
        motorDrive(topRight, -correctedPosition, power);
        while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())) {
            telemetry.addData("Corrected position", correctedPosition);
            telemetry.addData("Angle:", getAngle());
            telemetry.addData("topRight position", topRight.getTargetPosition() - topRight.getCurrentPosition());
            telemetry.addData("botRight position", bottomRight.getTargetPosition() - bottomRight.getCurrentPosition());
            telemetry.addData("topLeft position", topLeft.getTargetPosition() - topLeft.getCurrentPosition());
            telemetry.addData("botLeft position", bottomLeft.getTargetPosition() - bottomLeft.getCurrentPosition());
            telemetry.addData("topLeft power", topLeft.getPower());
            telemetry.addData("botLeft power", bottomLeft.getPower());
            telemetry.addData("topRight power", topRight.getPower());
            telemetry.addData("botRight power", bottomRight.getPower());

            telemetry.update();
        }
        sleep(250);
    }


    private void turnWithoutEncoder(double power) {
        topLeft.setPower(power);
        topRight.setPower(-power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
    }



}

