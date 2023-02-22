package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class Autonomustest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot();
        R.init(this);


        waitForStart();
        R.dr.move(60, 100);
        R.dr.move(60, -100);
    }
}
