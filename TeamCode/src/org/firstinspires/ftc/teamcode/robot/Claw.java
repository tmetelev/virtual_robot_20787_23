package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Robot R;
    Servo claw;
    HardwareMap hwd;

    public Claw(HardwareMap hwd, Robot R) {
        this.R = R;
        this.hwd = hwd;
        claw = hwd.get(Servo.class, "claw");
    }

    boolean flag = true;
    public void ClawMove(boolean position1) throws InterruptedException {
        if (position1) {
            flag = !flag;
            sleep(200);
        }
        if (!flag) {
            claw.setPosition(0);
        } else {
            claw.setPosition(1);
        }


    }
}
