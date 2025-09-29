package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionShooter extends LinearOpMode {

    DcMotor shooter;
    double x;
    double y;
    double r;
    double deltaR;
    double deltaSpeed;
    double power;
    double speed;
    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.get(DcMotor.class, "HoodShooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640,480))
                .build();

        waitForStart();

        deltaR = 2 - 1;
        deltaSpeed = 1;


        speed = deltaSpeed/deltaR;

        while(opModeIsActive()){

            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            if(tag.id == 20){
                x = tag.robotPose.getPosition().x;
                y = tag.robotPose.getPosition().y;

                r = Math.sqrt((x*x)+(y*y));
            }

            power = r*speed;

            if (gamepad1.right_trigger == 1){
                while(!gamepad1.b){
                    telemetry.addData("Power: ", power);
                    power = r*speed;
                    shooter.setPower(power);
                }
                shooter.setPower(0);
            }

        }

    }
}
