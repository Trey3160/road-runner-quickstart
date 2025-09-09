package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private DcMotor LFly;
    private DcMotor RFly;
    public CRServo Transfer;
    public static double speed = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        LFly = hardwareMap.get(DcMotor.class , "LFly");
        RFly = hardwareMap.get(DcMotor.class, "RFly");
        Transfer = hardwareMap.get(CRServo.class, "Transfer");
        RFly.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a){
                while (!gamepad1.b){
                    LFly.setPower(-0.5);
                    RFly.setPower(-0.5);
                    Transfer.setPower(-0.2);
                }
                LFly.setPower(0);
                RFly.setPower(0);
                Transfer.setPower(0);
            }

            if(gamepad1.right_trigger == 1){
                while (!gamepad1.b) {

                    LFly.setPower(speed);
                    RFly.setPower(speed);
                    sleep(2000);
                    Transfer.setPower(1);
                }
                LFly.setPower(0);
                RFly.setPower(0);
                Transfer.setPower(0);
            }
        }
    }
}
