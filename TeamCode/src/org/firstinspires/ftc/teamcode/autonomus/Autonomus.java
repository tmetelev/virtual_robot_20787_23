package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class Autonomus extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot();
        R.init(this);


        waitForStart();
        R.dr.followDirection(0.3, 1, 0, false, false);
        sleep(1500); // Робот едет 50 см/с
        R.dr.followDirection(0, 0,0, false, false);
    }
}
