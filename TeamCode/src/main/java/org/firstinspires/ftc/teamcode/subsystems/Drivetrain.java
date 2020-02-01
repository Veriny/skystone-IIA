package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    private DcMotor topRight;
    private DcMotor bottomRight;
    private DcMotor topLeft;
    private DcMotor bottomLeft;
    private Telemetry telemetry;
    private SkystoneContour skystoneContour;
//    private PIDCoefficients pidCoefficientDistance;
//    private PIDCoefficients pidCoefficientTurning;
    private ElapsedTime timeX;

    private static final int TICKS_PER_ROTATION = 679;
    private static final int WHEEL_DIAMETER = 4;
    private static final double BOT_DIAMETER = 17.5;
    private static final double BOT_CIRCUMFERENCE = Math.PI*BOT_DIAMETER;
    private ElapsedTime mRunTime;

    private double kP, kI, kD;

//    private double p_distance = 0.05;
//    private double i_distance = 0.0000004;
//    private double d_distance = 0;
//    private double p_turn = -0.0065;
//    private double i_turn = -0.00001;
//    private double d_turn = 0;


    public Drivetrain(DcMotor tl, DcMotor bl, DcMotor tr, DcMotor br, Boolean isAuto, Telemetry t) {
        this.topLeft = tl;
        this.bottomRight = br;
        this.topRight = tr;
        this.bottomLeft = bl;
        this.telemetry = t;

        if(isAuto) {
            topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
            topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            kP = 1.0;
            kI = 1.0;
            kD = 1.0;

//            pidCoefficientDistance = new PIDCoefficients(p_distance, i_distance, d_distance);
//            pidCoefficientTurning = new PIDCoefficients(p_turn, i_turn, d_turn);
        }
    }
    //김정은
    public void controls(Gamepad gp) {
        //TODO: Code Mecanum Bullshit
        float x = (float)(Math.pow(gp.left_stick_y, 3));
        float y = (float)(Math.pow(-gp.left_stick_x, 3));
        float z = (float)(Math.pow(-gp.right_stick_x, 3));
        if (gp.left_trigger != 0) {
            x /=3;
            y /=3;
            z/= 3;
        }
        bottomLeft.setPower(((-x)+(y)+(-z)));
        topLeft.setPower(((-x)+(-y)+(-z)));
        bottomRight.setPower(((x)+(y)+(-z)));
        topRight.setPower(((x)+(-y)+(-z)));
    }


    public void driveNo4C3(double distance, double power) {
        //TODO: Write method for driving
        //This code is written such that forward is positive.
        double position  = calculateTicks(distance);
        telemetry.addLine("Moved with position ticks: " + position);
        motorDrive(bottomLeft, position, power);
        motorDrive(bottomRight, position, power);
        motorDrive(topLeft, position, power);
        motorDrive(topRight, position, power);
        jigglyDed();
    }


    public void drive(double distance, double power) {
        //TODO: Write method for driving
        //This code is written such that forward is positive.
        double position  = calculateTicks(distance);
        telemetry.addLine("Moved with position ticks: " + position);
        motorDrive(bottomLeft, position, power);
        motorDrive(bottomRight, position, power);
        motorDrive(topLeft, position, power);
        motorDrive(topRight, position, power);
        jigglypuff();
    }

    public void driveNoDist(double power) {
        bottomLeft.setPower(-power);
        topLeft.setPower(-power);
        bottomRight.setPower(power);
        topRight.setPower(power);
    }

//    public void pidDrive(double distance, double maxPower) {
//        double position = calculateTicks(distance);
//        telemetry.addLine("PID: Moved with position ticks: " + position);
//
//            double error = distance - averageEncoders();
//            double totalError = 0;
//            if ((Math.abs(totalError + error) * i_distance < 1.0) &&
//                    (Math.abs(totalError + error) * i_distance > -1.0))
//                totalError += error;
//        // Perform the primary PID calculation
//        double resultPower = maxPower * (p_distance * error + i_distance * totalError);
//        motorDrive(bottomLeft, position, resultPower);
//        motorDrive(bottomRight, position, resultPower);
//        motorDrive(topLeft, position, resultPower);
//        motorDrive(topRight, position, resultPower);
//        jigglypuff();
//    }

//    public boolean skystoneIsCentered() {
//        boolean xCoordCentered = false;
//        boolean yCoordCentered = false;
//        ArrayList<Double> xPositions = skystoneContour.getContourXPos();
//        ArrayList<Double> yPositions = skystoneContour.getContourYPos();
//        if(xPositions.size() == 0 || yPositions.size() == 0) {
//            return false;
//        }
//        for(int contourX = 0; contourX < xPositions.size(); contourX++) {
//            if(xPositions.get(contourX) > 140 && 180 > xPositions.get(contourX)) {
//                xCoordCentered = true;
//            }
//        }
//        for(int contourY = 0; contourY < yPositions.size(); contourY++) {
//            if(yPositions.get(contourY) > 100 && 140 > yPositions.get(contourY)) {
//                yCoordCentered = true;
//            }
//        }
//        return xCoordCentered && yCoordCentered;
//    }

    public void strafe(double distance, double power) {
        //TODO: Write code for strafing
        //This code is written such that right is positive.
        double position  = calculateTicks(distance);
        telemetry.addLine("Moved with position ticks: " + position);
        motorDrive(bottomLeft, -position, power);
        motorDrive(bottomRight, position, power);
        motorDrive(topLeft, position, power);
        motorDrive(topRight, -position, power);
        jigglypuff();
    }

    public void turn(double degrees, double power) {
        //TODO: Write method for turning
        double rotations = degrees / 360 / 2.2;
        double position = calculateTicksRot(rotations * BOT_CIRCUMFERENCE);
        motorDrive(bottomLeft, position, power);
        motorDrive(bottomRight, -position, power);
        motorDrive(topLeft, position, power);
        motorDrive(topRight, -position, power);
        jigglypuff();
        telemetry.addLine("Moved with position ticks: " + position);
    }

    public void arcTurn(double degrees, int radius, double power, boolean leftSideOuter) {
        double distanceOuter = calculateTicks(degrees / 360 * (2 * Math.PI) * (BOT_DIAMETER + radius));
        double distanceInner = calculateTicks(degrees / 360 * (2 * Math.PI) * radius);
        double powerOuter = power;
        double powerInner = power * distanceInner / distanceOuter;
        if(leftSideOuter) {
            motorDrive(bottomLeft, distanceOuter, powerOuter);
            motorDrive(bottomRight, distanceInner, powerInner);
            motorDrive(topLeft, distanceOuter, powerOuter);
            motorDrive(topRight, distanceInner, powerInner);
            jigglypuff();
        }
        else {
            motorDrive(bottomLeft, distanceInner, powerInner);
            motorDrive(bottomRight, distanceOuter, powerOuter);
            motorDrive(topLeft, distanceInner, powerInner);
            motorDrive(topRight, distanceOuter, powerOuter);
            jigglypuff();
        }
    }

    public void REEEEEEE() {
        topLeft.setPower(1);
        bottomRight.setPower(1);
        topRight.setPower(1);
        bottomLeft.setPower(1);
    }

    public void seperateMotorPowerDrive(double TR, double BR, double TL, double BL) {
        topLeft.setPower(TL);
        topRight.setPower(TL);
        bottomLeft.setPower(BL);
        bottomLeft.setPower(BR);
    }

    public void driveForwardWithPID(double position) {
        //make sure to tune the PID to what we want it to be (kP, kI, kD values)
        double rotations = position / (WHEEL_DIAMETER * Math.PI);
        double ticks = rotations * TICKS_PER_ROTATION;
        mRunTime = new ElapsedTime();
        double[] power = {1, 1, 1, 1};
        double[] error = {rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION};
        double prevErrorTR, prevErrorTL, prevErrorBL, prevErrorBR;
        prevErrorTR = 0;
        prevErrorTL = 0;
        prevErrorBL = 0;
        prevErrorBR = 0;
        double[] integralError = {rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION};
        double[] derivativeError = {rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION, rotations * TICKS_PER_ROTATION};
        double prevTime = 0;
        mRunTime.reset();
        while (Math.abs(topRight.getCurrentPosition()) != Math.abs(ticks) || Math.abs(topLeft.getCurrentPosition()) != Math.abs(ticks) ||
                                    Math.abs(bottomLeft.getCurrentPosition()) != Math.abs(ticks) || Math.abs(bottomRight.getCurrentPosition()) != Math.abs(ticks)) {
            //The error array will store the errors as TR, BR, TL, BL respectively
            error[0] = ticks - topRight.getCurrentPosition();
            error[1] = ticks - bottomRight.getCurrentPosition();
            error[2] = ticks - topLeft.getCurrentPosition();
            error[3] = ticks - bottomLeft.getCurrentPosition();

            derivativeError[0] = (error[0] - prevErrorTR) / mRunTime.seconds() - prevTime;
            derivativeError[1] = (error[1] - prevErrorBR) / mRunTime.seconds() - prevTime;
            derivativeError[2] = (error[2] - prevErrorTL) / mRunTime.seconds() - prevTime;
            derivativeError[3] = (error[3] - prevErrorBL) / mRunTime.seconds() - prevTime;

            integralError[0] += error[0]*(mRunTime.seconds() - prevTime);
            integralError[1] += error[1]*(mRunTime.seconds() - prevTime);
            integralError[2] += error[2]*(mRunTime.seconds() - prevTime);
            integralError[3] += error[3]*(mRunTime.seconds() - prevTime);

            for (int i = 0; i < power.length; i++) {
                power[i] = Range.clip(kP * error[i] + kI * integralError[i] + kD * derivativeError[i], 1, -1);
            }
            seperateMotorPowerDrive(power[0], power[1], power[2], power[3]);
            prevTime = mRunTime.seconds();
            prevErrorTR = error[0];
            prevErrorBR = error[1];
            prevErrorTL = error[2];
            prevErrorBL = error[3];
        }
        mRunTime.reset();
        resetEncoders();
    }

    public double inchesMoved() {
        int ticks = averageEncoders();
        double inches = calculateInches(ticks);
        return inches;
    }

    public void resetEncoders() {
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void motorDrive(DcMotor motor, double ticks, double power) {
        //TODO: MotorDrive
        motor.setTargetPosition((int) ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private double calculateTicksRot(double inches) {
        return (inches / WHEEL_DIAMETER) * TICKS_PER_ROTATION;
    }

    private double calculateTicks(double inches) {
        return (inches / (WHEEL_DIAMETER * Math.PI) * TICKS_PER_ROTATION);
    }

    private double calculateInches(double ticks) {
        return (ticks * WHEEL_DIAMETER * Math.PI / TICKS_PER_ROTATION);
    }

    private int averageEncoders() {
        int average = (topLeft.getCurrentPosition() + topRight.getCurrentPosition() + bottomLeft.getCurrentPosition() + bottomRight.getCurrentPosition()) / 4;
        return average;
    }

    private void jigglypuff() {
        while ((topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy()) || (topLeft.isBusy() && topRight.isBusy() && bottomRight.isBusy()) ||
                (topLeft.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy()) || (topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())) {
            //do nothing
        }
        //I'm not sure if it's good practice to reset encoders every time we move - this may slow things down and we may have to change this in order to squeeze a few extra seconds out of auto in the future
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void jigglyDed(){
        while (topLeft.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || bottomRight.isBusy()){
        }
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        telemetry.addData("BL: ", bottomLeft.getCurrentPosition());
        telemetry.addData("TL: " , topLeft.getCurrentPosition());
        telemetry.addData("BR: " , bottomRight.getCurrentPosition());
        telemetry.addData("TR: " , topRight.getCurrentPosition());
        telemetry.update();
    }

    public void residentSleeper(int ms) {
        timeX = new ElapsedTime();
        timeX.reset();
        while (timeX.milliseconds() < ms){
            continue;
        }
    }
}