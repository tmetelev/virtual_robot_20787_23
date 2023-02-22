package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    Robot R;
    DcMotor elevator;
    HardwareMap hwd;

    public Elevator (HardwareMap hwd, Robot R) {
        this.R = R;
        this.hwd = hwd;
        elevator = hwd.get(DcMotor.class, "enc_x");

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void EvevatorMove(double power) {
        elevator.setPower(power);
    }
}
