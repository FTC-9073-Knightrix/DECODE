package org.firstinspires.ftc.teamcode.TeleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.mechanisms.MecanumDrive;

@TeleOp(name = "mecanum TeleOp")
public class MecanumFieldOrientedOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.runMecanumDrive(gamepad1.right_bumper, gamepad1.left_bumper,
                -gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x * .8, gamepad1.y);
    }

}
