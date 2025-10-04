package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeTest extends LinearOpMode {
   DcMotor intake;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a){
                while(!gamepad1.b){
                    intake.setPower(0.7);
                }
                intake.setPower(0);
            }
        }
    }
}
