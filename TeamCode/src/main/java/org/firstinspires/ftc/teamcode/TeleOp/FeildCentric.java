package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class FeildCentric extends OpMode {
    GoBildaPinpointDriver odo;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        FL = hardwareMap.get(DcMotor.class, "FLeft");
        BL = hardwareMap.get(DcMotor.class, "BLeft");
        FR = hardwareMap.get(DcMotor.class, "FRight");
        BR = hardwareMap.get(DcMotor.class, "BRight");

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //FL.setDirection(DcMotorSimple.Direction.REVERSE);


        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D StartingPos = new Pose2D(DistanceUnit.MM, 10,10, AngleUnit.DEGREES,0);
        odo.setPosition(StartingPos);

    }

    public void moveRobot(){

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];
        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FL.setPower(newWheelSpeeds[0]);
        FR.setPower(newWheelSpeeds[1]);
        BL.setPower(newWheelSpeeds[2]);
        BR.setPower(newWheelSpeeds[3]);


    }

    @Override
    public void loop() {
        moveRobot();

        Pose2D pos = odo.getPosition();
        odo.update();
    }
}
