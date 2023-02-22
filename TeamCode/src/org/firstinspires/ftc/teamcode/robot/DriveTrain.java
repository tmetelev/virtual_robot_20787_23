package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/***
 * Поля:
 * - Моторы
 *
 * - Датчики
 *
 * Методы:
 * - Общий метод для подачи мощности на все моторы
 *
 * - Работы с датчиками
 */
public class DriveTrain {
    Robot R;
    DcMotor bl, fl, fr, br;
    HardwareMap hwd;

    public static double kP = 1;
//    public static double TPR = 1440;

    public DriveTrain (HardwareMap hwd, Robot R) {
        this.R = R;
        this.hwd = hwd;
        bl = hwd.get(DcMotor.class, "back_left_motor");
        fl = hwd.get(DcMotor.class, "front_left_motor");
        fr = hwd.get(DcMotor.class, "front_right_motor");
        br = hwd.get(DcMotor.class, "back_right_motor");

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // init IMU
    }


    public void GoAngle(int goal) {
        while (Math.abs(goal - R.getAngle()) > 5) {
            double e = goal - R.getAngle();
            setPower(-e, -e, e, e, 0.2);
            R.tele.addData("angle", R.getAngle());
            R.tele.update();
        }
    }

    public static int WHEEL_RADIUS = 5;         // Радиус колеса
    public static int TPR = 1440;               // Для моторов Tetrix 60:1

    public static double kPx = 0.1;             // Пропорциональный коэффициент
    public static double kPy = 0.1;             // Пропорциональный коэффициент

    public static int ZONE_THRESHOLD = 50;     // Погрешнать в тиках, до которой должна уменьшиться ошибка
    public static int TIME_THRESHOLD = 10000;     // Ограничение цикла по времени

    public void move(double x, double y) {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);     // Обнуление значения энкодеров
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double cos = Math.cos(Math.toRadians(45));              // Подссчет sin(45) и cos(45)
        double sin = Math.sin(Math.toRadians(45));

        double x1 = x * cos - y * sin;                          // Переход в С.К. робота
        double y1 = x * sin + y * cos;

        x1 = (x1 * TPR) / (2 * Math.PI * WHEEL_RADIUS);         // Получение цели в тиках
        y1 = (y1 * TPR) / (2 * Math.PI * WHEEL_RADIUS);

        double errx = x1;                                       // Инициализация ошибки
        double erry = y1;

        ElapsedTime t = new ElapsedTime();                      // Инициализация таймера
        double t0 = t.milliseconds();

        while ((Math.abs(errx) > ZONE_THRESHOLD || Math.abs(erry) > ZONE_THRESHOLD) && t.milliseconds() - t0 < TIME_THRESHOLD) {
            double x0 = fl.getCurrentPosition();                // Считывание позиции робота
            double y0 = bl.getCurrentPosition();

            errx = x1 - x0;                                     // Вычисление ошибки
            erry = y1 - y0;

            double px = errx * kPx;                             // Получение упр. сигналов
            double py = erry * kPy;

            if (Math.abs(px) > 1 && Math.abs(px) > Math.abs(py)) {      // Избавление от избыточных
                py /= Math.abs(px);                                     // значений
                px /= Math.abs(px);
            }
            if (Math.abs(py) > 1 && Math.abs(py) > Math.abs(px)) {
                px /= Math.abs(py);
                py /= Math.abs(py);
            }

            setPower(py, px, py, px);                                 // Подача на моторы

            R.tele.addData("x", errx);                         // Телеметрия
            R.tele.addData("y", erry);
            R.tele.update();
        }

        setPower(0, 0, 0, 0);                            // Остановка тележки в конце
    }

    public void GoPoint(int CmX, int CmY) {
        double r = 5;
        double spt = (2 * Math.PI * r) / TPR;
        double tps = Math.pow(spt, -1);
        while (Math.abs( tps * CmX - R.dr.bl.getCurrentPosition()) > 100) {
            double e = tps * CmX - R.dr.bl.getCurrentPosition();
            e *= kP;
            followDirection(e, 0, 0, false, false);
            R.tele.addData("ticks motor", R.dr.bl.getCurrentPosition());
            R.tele.update();
        }
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(tps * CmY - R.dr.fr.getCurrentPosition()) > 100) {
            double e = tps * CmY - R.dr.fr.getCurrentPosition();
            e *= kP;
            followDirection(0, e, 0, false, false);
            R.tele.addData("tick motor", R.dr.fr.getCurrentPosition());
            R.tele.update();
        }
    }



    public void setPower(double bl, double fl, double fr, double br) {
        this.bl.setPower(bl);
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.br.setPower(br);
    }

    public void setPower(double bl, double fl, double fr, double br, double k) {
        this.bl.setPower(bl * k);
        this.fl.setPower(fl * k);
        this.fr.setPower(fr * k);
        this.br.setPower(br * k);
    }

    // t < 0 - left t > 0 - right

    public void followDirection (double x, double y, double t, boolean isSlow, boolean isSlower) {
        double sin = Math.sin(Math.toRadians(45));
        double cos = sin;

        double tX = 0;
        double tY = 0;
        double k = 1;

        double len = x * x + y * y;

        tY = (y / sin - x / cos) / 2;
        tX = (y / sin + x / cos) / 2;

        if (isSlow) {
            k = 0.6;
        }
        if (isSlower) {
            k = 0.3;
        }

        if (Math.abs(tX) >= Math.abs(tY) && tX != 0) {
            tY /= Math.abs(tX);
            tX /= Math.abs(tX);
        } else if (Math.abs(tX) <= Math.abs(tY) && tY != 0) {
            tX /= Math.abs(tY);
            tY /= Math.abs(tY);
        }
        tX *= len * k;
        tY *= len * k;

        if (len < 0.002) {
            setPower(t, t, -t, -t);
        } else {
            setPower(
                    tX + t,
                    tY + t,
                    tX - t,
                    tY - t
            );
        }


    }

}
