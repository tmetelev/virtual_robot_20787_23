package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.omg.CORBA.PUBLIC_MEMBER;

/***
 * Поля:
 * - Модули, сенсоры, интерфейс для видеозрения, интерфейс для управления колесной базы (по жеданию)
 * - LinearOpMode
 * - HardwareMap
 * - Telemetry
 * - Gamepad
 * - Состония робота
 * - Потоки
 * Методы:
 * - Инициализации (полей, модулей, видеозрения, вспомогательных классов)
 * - Комплекосное несколькими модулями
 * - Цикл управлемого периода
 */
public class Robot {
    BNO055IMU imu;
    public DriveTrain dr;
    HardwareMap hwd;
    Telemetry tele;
    Gamepad gamepad1, gamepad2;
    LinearOpMode li;
    Elevator elv;
    Claw claw;

    public void init(LinearOpMode li) throws InterruptedException {
        this.li = li;
        hwd = li.hardwareMap;
        tele = li.telemetry;
        gamepad1 = li.gamepad1;
        gamepad2 = li.gamepad2;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwd.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
//        tele.addData("Wait", "Calibrating");
//        tele.update();
//        while (!imu.isGyroCalibrated()) {}
//        tele.addData("Ok", "Calibrated");
//        tele.update();


        dr = new DriveTrain(hwd, this);
//        elv = new Elevator(hwd, this);
//        claw = new Claw(hwd, this);
        tele.addLine("Init completed");
        tele.update();
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    public void teleLoop() throws InterruptedException {
        while (li.opModeIsActive()) {
            dr.followDirection(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.a, gamepad1.b);
            tele.addData("angle", getAngle());
            tele.update();

        }
    }
}
