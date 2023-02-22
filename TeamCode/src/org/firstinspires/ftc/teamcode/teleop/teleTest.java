package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class teleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot();
        R.init(this);
        
        waitForStart();

        R.teleLoop();



    }
}
