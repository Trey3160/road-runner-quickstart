package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
@Config
public class VisionShooter extends LinearOpMode {
    DcMotor BL;
    DcMotor BR;
    DcMotor FL;
    DcMotor FR;
    WebcamName Webcam1;
    FtcDashboard dashboard;
    DcMotor shooter;
    GoBildaPinpointDriver odo;
    double x;
    double y;
    double hypot;
    double yaw;
    double power;
    double speed;
    double theta;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        shooter = hardwareMap.get(DcMotor.class, "HoodShooter");
        Webcam1 = hardwareMap.get(WebcamName.class, "Webcam1");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        BL = hardwareMap.get(DcMotor.class,"BLeft");
        BR = hardwareMap.get(DcMotor.class,"BRight");
        FL = hardwareMap.get(DcMotor.class,"FLeft");
        FR = hardwareMap.get(DcMotor.class,"FRight");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        theta = 25;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640,480))
                .setAutoStartStreamOnBuild(true)
                .build();


        waitForStart();


        waitForStart();
        while(opModeIsActive()){

            dashboard.startCameraStream(visionPortal,720);

            if (gamepad1.a){
                BL.setPower(0.5);
            }
            if(gamepad1.b){
                BR.setPower(0.5);
            }
            if(gamepad1.x){
                FL.setPower(0.5);
            }
            if(gamepad1.y){
                FR.setPower(0.5);
            }else{
                BL.setPower(0);
                BR.setPower(0);
                FR.setPower(0);
                FL.setPower(0);
            }



            if (gamepad1.right_trigger == 1){
                while(!gamepad1.b){
                    power = CalcPower();
                    //telemetry.addData("Power: ", power);
                    shooter.setPower(power);
                    //telemetry.update();
                }
                shooter.setPower(0);
            }
            if (gamepad1.left_trigger == 1){
                while(!gamepad1.b){
                    power = CalcPower();
                    //telemetry.addData("Power: ", power);
                    shooter.setPower(power);
                    aim();
                    telemetry.addLine("\uD83D\uDFE9");
                    telemetry.update();
                }
                shooter.setPower(0);
            }

        }

    }
    public double CalcPower(){
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            if (tag.id == 20) {
                x = tag.rawPose.x;
                y = tag.rawPose.y;
            } else {
                x = 0;
                y = 0;
            }
            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("hypot: ", hypot);
            hypot = Math.sqrt(x*x+y*y);
            speed = 0.0072*hypot+0.6324;
            telemetry.update();
        }
        return speed;
    }
    public void aim() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            if (tag.id == 20) {
                yaw = tag.robotPose.getOrientation().getYaw();
            } else {
                yaw = 0;
            }
            while (yaw > 10 && yaw < -10) {
                if(yaw > 10){
                    BL.setPower(-0.3);
                    BR.setPower(0.3);
                    FL.setPower(-0.3);
                    FR.setPower(0.3);
                }if(yaw < -10){
                    BL.setPower(0.3);
                    BR.setPower(-0.3);
                    FL.setPower(0.3);
                    FR.setPower(-0.3);
                }
            }

            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);
        }
    }
}
