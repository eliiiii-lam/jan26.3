package org.firstinspires.ftc.teamcode;
import android.graphics.drawable.GradientDrawable;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class IMUtestywesty extends LinearOpMode{

    DcMotor rightDrive;
    DcMotor leftDrive;

    DcMotor elbow;
    DcMotor elbow2;

    Servo clawL;
    Servo clawR;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.Json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftDrive = hardwareMap.dcMotor.get("fL");
        rightDrive = hardwareMap.dcMotor.get("fR");
        leftDrive = hardwareMap.dcMotor.get("bL");
        rightDrive = hardwareMap.dcMotor.get("bR");

        elbow = hardwareMap.dcMotor.get("elbow");
        elbow2 = hardwareMap.dcMotor.get("elbow2");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);


        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(DcMotor.Direction.FORWARD);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        waitForStart();


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);





    }

    void resetEncoders(){
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    void driveForward(double power, double ticks, float targetAngle) throws InterruptedException{
        double leftPower;
        double rightPower;
        rightDrive.setPower(power);
        leftDrive.setPower(power);


        while (leftDrive.getCurrentPosition()  < ticks && rightDrive.getCurrentPosition()  < ticks && opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle<targetAngle) {
                rightPower = power + 0.5;
                 leftPower = power - 0.5;
            }else if (angles.firstAngle > targetAngle) {
                rightPower = power - 0.5;
                leftPower = power + 0.5;
            } else {
                leftPower = power;
                rightPower = power;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        resetEncoders();
    }


    void turn(double turnAngle, double timeoutS) {
        sleep(2000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double speed = .5;
        double oldDegrees = turnAngle;
        double scaledSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;

        if(targetHeading<-180) {targetHeading += 360;}
        if (targetHeading > 180) {targetHeading -= 360;}

        double degrees = ((Math.signum(angles.firstAngle-targetHeading) + 1) / 2) * (360-Math.abs(angles.firstAngle-targetHeading)) + (Math.signum(targetHeading - angles.firstAngle) + 1) / 2*Math.abs(angles.firstAngle-targetHeading);
        resetRuntime();

      //  while (opModeIsActive() && time < timeoutS && degrees>1&& oldDegrees)
    }





}



