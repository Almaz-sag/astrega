package org.firstinspires.ftc.teamcode.actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "DINAHUY", group = "Drive")
public class DINAHUY extends LinearOpMode {

    public static double nominalVoltage = 13.0; // напряжение, при котором настраивал kF

    private VoltageSensor battery;
    private double voltage = 12.0;
    private long lastVoltageRead = 0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;

    // ===== LAUNCHER =====
    private DcMotorEx launcher, launcher2;
    private Servo stopper, ramp;

    // ===== TURRET =====
    private Servo tur1, tur2;
    private GoBildaPinpointDriver pinpoint;

    // ===== LAUNCHER PIDF =====
    public static double kF = 0.000365;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double filterAlpha = 0; // сглаживание скорости
    public static double iMax = 0.15;       // лимит интеграла (в мощности)
    public static double iZone = 75;       // интеграл копится только при малой ошибке

    private double integral = 0, lastError = 0;
    private double filteredVel = 0, lastTarget = 0;
    private long lastTime = 0;
    private double targetVelocity;

    public static double OPEN = 0;
    public static double CLOSE = 0.15;
    public static double RAMP = 0.4;

    // ===== FIELD =====
    public static double StartX = 6.5;
    public static double StartY = 7;

    public static double perpX = -5.038070866;
    public static double parY = 2.96708661;

    public static double turretOffsetX = 0.23531496; // inches вперед
    public static double turretOffsetY = 0; // если по центру


    public static double turmin = 0.0;
    public static double turmax = 1.0;

    public double lgoalX;
    public static double lgoalY = 84;

    public double rgoalX;
    public static double rgoalY = 60;
    public static double pgoalX = 64;
    public static double cgoalX = 62;
    public static double mgoalX = 60;
    public static double fgoalX = 57;

    public static double p = 64;
    public static double c = 62;
    public static double m = 60;
    public static double f = 57;

    public double post;
    public double pos2t;
    public double tV;
    public static double pos0 = 0.167;
    public static double pos90 = 0.5;
    public static double posn90 = 0.5;
    public static double posn180 = 0.833;
    public static double posfl = 0.595;
    public static double posfr = 0.39;
    public static double tVc = 2135;
    public static double tVf = 2150;


    public double goalX;
    public double goalY;
    public static double d1 = 30;
    public static double d2 = 35;
    public static double d3 = 40;
    public static double d4 = 45;
    public static double d5 = 50;
    public static double d6 = 55;
    public static double d7 = 60;
    public static double d8 = 65;
    public static double d9 = 70;



    public static double v1 = 2100;
    public static double v2 = 2100;
    public static double v3 = 2100;
    public static double v4 = 2110;
    public static double v5 = 2125;
    public static double v6 = 2150;
    public static double v7 = 2200;
    public static double v8 = 2215;
    public static double v9 = 2235;
    public static int LEFT_TAG = 35;
    public static int RIGHT_TAG = 32;
    public int AIM;
    private boolean track = true;
    public static int dega = 270;

    private double normalize(double a) {
        while (a > Math.PI)
            a -= 2 * Math.PI;

        while (a < -Math.PI)
            a += 2 * Math.PI;

        return a;
    }

    public static double centerOffsetDeg = -1.5;

    private double angleToServo(double angle) {
        angle = normalize(angle);

        double range = Math.toRadians(dega);
        double servo = 0.5 + (angle / range) + Math.toRadians(centerOffsetDeg) / range;

        return Range.clip(servo, 0.0, 1.0);
    }

    public double getVelocityFromTable(double distance) {

        double[] distances = {
                d1, d2, d3, d4, d5,
                d6, d7, d8, d9
        };

        double[] velocities = {
                v1, v2, v3, v4, v5,
                v6, v7, v8, v9
        };

        // ниже минимума
        if (distance <= distances[0])
            return velocities[0];

        // выше максимума
        if (distance >= distances[distances.length - 1])
            return velocities[velocities.length - 1];

        // ищем промежуток
        for (int i = 0; i < distances.length - 1; i++) {

            if (distance >= distances[i] && distance <= distances[i + 1]) {

                double d1 = distances[i];
                double d2 = distances[i + 1];

                double v1 = velocities[i];
                double v2 = velocities[i + 1];

                // линейная интерполяция
                double t = (distance - d1) / (d2 - d1);

                return v1 + t * (v2 - v1);
            }
        }

        return velocities[0]; // fallback
    }

    public void updateTurret() {

        Pose2D pos = pinpoint.getPosition();

        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.RADIANS);

        // ===== APPLY TURRET OFFSET =====
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double turretX =
                robotX + turretOffsetX * cos - turretOffsetY * sin;

        double turretY =
                robotY + turretOffsetX * sin + turretOffsetY * cos;

        // ===== TARGET VECTOR =====
        double dx = goalX - turretX;
        double dy = goalY - turretY;

        double targetAngle;

        targetAngle = Math.atan2(dy, dx);

        // ===== RELATIVE ANGLE =====
        double relativeAngle = normalize(targetAngle - robotHeading);

        // ===== SERVO MAPPING =====
        double servoPos = angleToServo(relativeAngle);

        // ===== SAFETY CLAMP =====
        servoPos = Range.clip(servoPos, turmin, turmax);

        tur1.setPosition(servoPos);
        tur2.setPosition(servoPos);
    }

    public double getTurretError() {

        Pose2D pos = pinpoint.getPosition();

        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double turretX =
                robotX + turretOffsetX * cos - turretOffsetY * sin;

        double turretY =
                robotY + turretOffsetX * sin + turretOffsetY * cos;

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        double targetAngle = Math.atan2(dy, dx);

        double relativeAngle =
                normalize(targetAngle - robotHeading);

        return relativeAngle; // В РАДИАНАХ!
    }

    public void moveRobot() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * 0.7;

        double heading =
                pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double globalForward =
                forward * cos - strafe * sin;

        double globalStrafe =
                forward * sin + strafe * cos;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] =
                globalForward + globalStrafe + rotate;

        newWheelSpeeds[1] =
                globalForward - globalStrafe - rotate;

        newWheelSpeeds[2] =
                globalForward - globalStrafe + rotate;

        newWheelSpeeds[3] =
                globalForward + globalStrafe - rotate;

        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();

        tur1 = hardwareMap.get(Servo.class, "tur1");
        tur2 = hardwareMap.get(Servo.class, "tur2");
        ramp = hardwareMap.get(Servo.class, "ramp");

        pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "pinpoint"
                );

        frontLeft =
                hardwareMap.get(DcMotor.class, "leftFront");

        frontRight =
                hardwareMap.get(DcMotor.class, "rightFront");

        backLeft =
                hardwareMap.get(DcMotor.class, "leftBack");

        backRight =
                hardwareMap.get(DcMotor.class, "rightBack");

        intake1 =
                hardwareMap.get(DcMotor.class, "intake1");

        intake2 =
                hardwareMap.get(DcMotor.class, "intake2");

        launcher =
                hardwareMap.get(DcMotorEx.class, "launcher");

        launcher2 =
                hardwareMap.get(DcMotorEx.class, "launcher2");

        stopper =
                hardwareMap.get(Servo.class, "stopper");

        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake1.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake2.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        backLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        launcher2.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        pinpoint.setOffsets(
                parY,
                perpX,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        Pose2D startPose =
                new Pose2D(
                        DistanceUnit.INCH,
                        StartX,
                        StartY,
                        AngleUnit.DEGREES,
                        0
                );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Init: resetting pinpoint...");
        telemetry.addLine("pls dont move/start the robot");
        telemetry.addLine("for 3-5 seconds");
        telemetry.update();

        ((ServoImplEx) tur1).setPwmRange(new PwmControl.PwmRange(500, 2500));
        ((ServoImplEx) tur2).setPwmRange(new PwmControl.PwmRange(500, 2500));

        waitForStart();

        pinpoint.setPosition(startPose);

        while (opModeIsActive()) {

            pinpoint.update();

            // ===== DISTANCE =====
            Pose2D pos = pinpoint.getPosition();

            double RHead =
                    pos.getHeading(AngleUnit.RADIANS);

            double robotX =
                    pos.getX(DistanceUnit.INCH);

            double robotY =
                    pos.getY(DistanceUnit.INCH);

            if (robotY > 72) {
                AIM = LEFT_TAG;
                if (robotX > 48 && robotX < 72) {
                    lgoalX = mgoalX;
                } else if (robotX < 48 && robotX > 24) {
                    lgoalX = cgoalX;
                } else if (robotX > 72) {
                    lgoalX = fgoalX;
                } else if (robotX < 24) {
                    lgoalX = pgoalX;
                }
                goalX = lgoalX;
                goalY = lgoalY;
            } else if (robotY < 72) {
                AIM = RIGHT_TAG;
                if (robotX > 48 && robotX < 72) {
                    rgoalX = m;
                } else if (robotX < 48 && robotX > 24) {
                    rgoalX = c;
                } else if (robotX > 72) {
                    rgoalX = f;
                } else if (robotX < 24) {
                    rgoalX = p;
                }
                goalX = rgoalX;
                goalY = rgoalY;
            }

            double cos =
                    Math.cos(pos.getHeading(AngleUnit.RADIANS));

            double sin =
                    Math.sin(pos.getHeading(AngleUnit.RADIANS));

            double turretX =
                    robotX + turretOffsetX * cos - turretOffsetY * sin;

            double turretY =
                    robotY + turretOffsetX * sin + turretOffsetY * cos;

            double dx = goalX - turretX;
            double dy = goalY - turretY;

            double distance =
                    Math.sqrt(dx * dx + dy * dy);

            moveRobot();


            if (gamepad1.y) {
                ramp.setPosition(RAMP);
            } else if (gamepad1.a) {
                ramp.setPosition(1);
            }


            if (gamepad2.right_bumper) {
                track = true;
            } else {
                track = false;
            }


            if (gamepad1.dpad_right) {
                Pose2D reset1 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                6.5,
                                7,
                                AngleUnit.DEGREES,
                                0
                        );

                pinpoint.setPosition(reset1);
            }

            if (gamepad1.dpad_left) {

                Pose2D reset2 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                6.5,
                                134.6,
                                AngleUnit.DEGREES,
                                0
                        );

                pinpoint.setPosition(reset2);
            }

            if (gamepad1.dpad_down) {

                Pose2D reset3 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                6.5,
                                58.9,
                                AngleUnit.DEGREES,
                                0
                        );

                pinpoint.setPosition(reset3);
            }

            double h = normalize(RHead);   // Pinpoint уже отдает от -180° до 180°, но так надежнее

            if (h >= Math.toRadians(-45) && h < Math.toRadians(45)) {
                post = pos0;
                pos2t = posn180;// примерно 0°
            } else if (h >= Math.toRadians(45) && h < Math.toRadians(135)) {
                post = pos90;
                pos2t = posn90; // 90° влево
            } else if (h >= Math.toRadians(-135) && h < Math.toRadians(-45)) {
                post = posn90;
                pos2t = pos90;// 90° вправо
            } else {
                post = posn180;
                pos2t = pos0;// около 180° (от 135° через 180° до -135°)
            }


            if(track){
                updateTurret();
            } else {
                if(gamepad1.right_bumper){
                    targetVelocity = tVc;
                    if(robotY > 72){
                        tur1.setPosition(post);
                        tur2.setPosition(post);
                    } else{
                        tur1.setPosition(pos2t);
                        tur2.setPosition(pos2t);
                    }
                } else if (gamepad1.left_bumper) {
                    targetVelocity = tVf;
                    if(robotY > 72){
                        tur1.setPosition(posfl);
                        tur2.setPosition(posfl);
                    } else{
                        tur1.setPosition(posfr);
                        tur2.setPosition(posfr);
                    }
                } else {
                    tur1.setPosition(0.5);
                    tur2.setPosition(0.5);
                }
            }

            // ===== LAUNCHER PIDF =====
            double rawVelocity =
                    launcher.getVelocity();

            filteredVel =
                    filterAlpha * filteredVel
                            + (1 - filterAlpha) * rawVelocity;

            double currentVelocity = filteredVel;

            // при смене цели сбрасываем интеграл
            if (Math.abs(targetVelocity - lastTarget) > 50) {
                integral = 0;
            }
            lastTarget = targetVelocity;

            double error =
                    targetVelocity - filteredVel;

            long now =
                    System.nanoTime();

            double dt =
                    (lastTime == 0)
                            ? 0
                            : (now - lastTime) / 1e9;

            lastTime = now;

            if (dt > 0) {
                if (Math.abs(error) < iZone) {
                    integral += error * dt * kI;
                    integral = Range.clip(integral, -iMax, iMax);
                } else {
                    integral = 0;
                }
            }

            double derivative =
                    (dt > 0)
                            ? (error - lastError) / dt
                            : 0;

            lastError = error;

            // читаем напряжение раз в 100 мс (вызов к хабу небыстрый) и слегка сглаживаем
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastVoltageRead > 100) {
                voltage = 0.8 * voltage + 0.2 * battery.getVoltage();
                lastVoltageRead = nowMs;
            }

            double voltageScale = nominalVoltage / Math.max(voltage, 8.0);

            double output =
                    kF * targetVelocity * voltageScale
                            + kP * error
                            + integral
                            + kD * derivative;

            output =
                    Range.clip(
                            output,
                            -1.0,
                            1.0
                    );

            launcher.setPower(output);
            launcher2.setPower(output);

            // ===== INTAKE =====
            if (
                    Math.abs(
                            gamepad2.left_trigger
                                    - gamepad2.right_trigger
                    ) > 0.1
            ) {

                double p =
                        gamepad2.left_trigger
                                - gamepad2.right_trigger;

                intake1.setPower(p);
                intake2.setPower(-p);

                stopper.setPosition(OPEN);

            } else {

                double p =
                        gamepad1.left_trigger
                                - gamepad1.right_trigger;

                intake1.setPower(p);
                intake2.setPower(-p);

                stopper.setPosition(CLOSE);
            }

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData(
                    "H",
                    Math.toDegrees(RHead)
            );
            telemetry.addData(
                    "Distance",
                    distance
            );
            telemetry.addData(
                    "Output",
                    output
            );
            telemetry.addData(
                    "error",
                    error
            );
            telemetry.addData(
                    "curVel",
                    currentVelocity
            );
            telemetry.addData(
                    "tarVel",
                    targetVelocity
            );
            telemetry.addData("integral", integral);

            telemetry.addData("voltage", voltage);

            telemetry.update();
        }
    }
}