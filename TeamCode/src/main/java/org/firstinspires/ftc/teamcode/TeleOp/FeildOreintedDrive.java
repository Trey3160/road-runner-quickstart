package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class FeildOreintedDrive extends LinearOpMode {
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor FRight;
    private DcMotor BRight;






    @Override
    public void runOpMode() {
        telemetry.addLine("Initalizd");




        //Init's the motors
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        //Set Direction
        //FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //BLeft.setDirection(DcMotor.Direction.REVERSE);
        FRight.setDirection(DcMotor.Direction.REVERSE);
        BRight.setDirection(DcMotor.Direction.REVERSE);
        //zeroPower Behavior(Coast to stop/Imediate Stop)
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);


        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Rotation:", -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            if (gamepad1.a){
                BLeft.setPower(0.5);
            }
            if(gamepad1.x){
                FRight.setPower(0.5);
            }
            if(gamepad1.y){
                FLeft.setPower(0.5);
            }
            if(gamepad1.b){
                BRight.setPower(0.5);
            }


            double rotation = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;


            if (rotation >= 45 &&rotation <= 135 || rotation <= -45 && rotation >= -135){
                lx = -gamepad1.left_stick_x;
            }




            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);


            double power = 0.2 + (0.6 * gamepad1.right_trigger);


            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);






            if(rotation >= 46 && rotation <= 56 || rotation <=135 && rotation >= 106 || rotation >= -68 && rotation < -48 || rotation <= 135 && rotation >= 107){


                if(gamepad1.right_stick_x < 0){
                    FLeft.setPower(-0.25);
                    BLeft.setPower(-0.25);
                    FRight.setPower(0.25);
                    BRight.setPower(0.25);
                }else{
                    FLeft.setPower(0.25);
                    BLeft.setPower(0.25);
                    FRight.setPower(-0.25);
                    BRight.setPower(-0.25);
                }
            }
            else if (rotation >= 45 && rotation <= 135 || rotation <= -45 && rotation >= -135) {
                rx = -gamepad1.right_stick_x;
                if(gamepad1.right_trigger > 0.2){
                    double power1 = -((adjustedLy + adjustedLx + rx) / max) * power;
                    double power2 = -((adjustedLy - adjustedLx + rx) / max) * power;
                    double power3 = -((adjustedLy - adjustedLx - rx) / max) * power;
                    double power4 = -((adjustedLy + adjustedLx - rx) / max) * power;
                    if(rotation >= 45 && rotation <= 135){
                        if (gamepad1.left_stick_y == -1){
                            FRight.setPower(gamepad1.right_trigger);
                            FLeft.setPower(-gamepad1.right_trigger);
                            BRight.setPower(-gamepad1.right_trigger);
                            BLeft.setPower(gamepad1.right_trigger);
                        }else if (gamepad1.left_stick_y == 1){
                            FRight.setPower(-gamepad1.right_trigger);
                            FLeft.setPower(gamepad1.right_trigger);
                            BRight.setPower(gamepad1.right_trigger);
                            BLeft.setPower(-gamepad1.right_trigger);
                        }if (gamepad1.left_stick_x == -1){
                            FRight.setPower(-gamepad1.right_trigger);
                            FLeft.setPower(-gamepad1.right_trigger);
                            BRight.setPower(-gamepad1.right_trigger);
                            BLeft.setPower(-gamepad1.right_trigger);
                        }else if (gamepad1.left_stick_x == 1){
                            FRight.setPower(gamepad1.right_trigger);
                            FLeft.setPower(gamepad1.right_trigger);
                            BRight.setPower(gamepad1.right_trigger);
                            BLeft.setPower(gamepad1.right_trigger);
                        }else{
                            FLeft.setPower(power1);
                            BLeft.setPower(power2);
                            FRight.setPower(power3);
                            BRight.setPower(power4);
                        }
                    }else if(rotation <= -45 && rotation >= -135){
                        if (gamepad1.left_stick_y == 1){
                            FRight.setPower(gamepad1.right_trigger);
                            FLeft.setPower(-gamepad1.right_trigger);
                            BRight.setPower(-gamepad1.right_trigger);
                            BLeft.setPower(gamepad1.right_trigger);
                        }else if (gamepad1.left_stick_y == -1){
                            FRight.setPower(-gamepad1.right_trigger);
                            FLeft.setPower(gamepad1.right_trigger);
                            BRight.setPower(gamepad1.right_trigger);
                            BLeft.setPower(-gamepad1.right_trigger);
                        }else {


                            FLeft.setPower(power1);
                            BLeft.setPower(power2);
                            FRight.setPower(power3);
                            BRight.setPower(power4);
                        }
                    }else{
                        FLeft.setPower(power1);
                        BLeft.setPower(power2);
                        FRight.setPower(power3);
                        BRight.setPower(power4);
                    }
                }


            }else {


                FLeft.setPower(((adjustedLy + adjustedLx + rx) / max) * power);
                BLeft.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
                FRight.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
                BRight.setPower(((adjustedLy + adjustedLx - rx) / max) * power);
            }
            telemetry.addData("Front Left Motor Power: ", FLeft.getPower());
            telemetry.addData("Front Right Motor Power: ", FRight.getPower());
            telemetry.addData("Back Right Motor Power: ", BRight.getPower());
            telemetry.addData("Back Left Motor Power: ", BLeft.getPower());
            telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}

