package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FeildDrive extends LinearOpMode {
    DcMotor FLeft;
    DcMotor BLeft;
    DcMotor FRight;
    DcMotor BRight;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        BLeft = hardwareMap.get(DcMotor.class,"BLeft");
        FRight = hardwareMap.get(DcMotor.class,"FRight");
        BRight = hardwareMap.get(DcMotor.class,"BRight");

        imu = hardwareMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOreintation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOreintation));
        FRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            driveFeild(forward,strafe,rotate);

        }
    }
    public void drive(double forward, double strafe,double rotate){
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward + strafe + rotate;
        double frontRightPower = forward + strafe + rotate;
        double backRightPower = forward + strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower,Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower,Math.abs(backLeftPower));
        maxPower = Math.max(maxPower,Math.abs(frontRightPower));
        maxPower = Math.max(maxPower,Math.abs(backRightPower));

        FLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        BLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        FRight.setPower(maxSpeed * (frontRightPower / maxPower));
        BRight.setPower(maxSpeed * (backRightPower / maxPower));
    }
    public void driveFeild(double forward, double strafe,double rotate){
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe,forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r* Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward,newStrafe,rotate);
    }
}
