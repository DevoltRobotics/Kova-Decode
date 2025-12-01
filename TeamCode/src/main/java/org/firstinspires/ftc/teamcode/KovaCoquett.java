package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public abstract class KovaCoquett extends OpMode {

    // ------------------- ROBOT & SUBSYSTEMS -------------------
    protected final HardwareCoquett robot;
    protected final Intake intake;

    private static final double RGB_OFF    = 0.0;
    private static final double RGB_RED    = 0.277;
    private static final double RGB_GREEN  = 0.500;
    private static final double RGB_VIOLET = 0.722;
    private static final double RGB_WHITE  = 1.0;
    private static final double RGB_PINK = 0.85;
    private static final double RGB_SPEC_MIN = RGB_RED;    // 0.277
    private static final double RGB_SPEC_MAX = RGB_VIOLET; // 0.722
    private static final int COLOR_MIN_BRIGHTNESS = 80;
    private static final double COLOR_DOMINANT_RATIO = 1.4;
    private static final double RGB_CYCLE_PERIOD = 4.0;

    private final ElapsedTime rgbCycleTimer = new ElapsedTime();

    // --------------------------------------------------------------

    public KovaCoquett(Alliance alliance) {
        robot = new HardwareCoquett(alliance);
        intake = new Intake(robot);
    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        rgbCycleTimer.reset();

        // Show pink while in INIT
        if (robot.light != null) {
            robot.light.setPosition(RGB_PINK);
        }
    }

    @Override
    public void start() {
        robot.follower.startTeleOpDrive(true);
        rgbCycleTimer.reset();
    }

    @Override
    public void loop() {
        // ------------------- DRIVE -------------------
        if (!gamepad1.b) {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.left_stick_x * 0.5,
                    -gamepad1.right_stick_x * 0.5,
                    false
            );
            telemetry.addData("Modo lento", "Activado");
            robot.light.setPosition(0.277);
        } else {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            telemetry.addData("Modo lento", "Desactivado");
            robot.light.setPosition(0.5);
        }

        if (gamepad1.dpadUpWasPressed()) {
            robot.follower.setPose(new Pose());
        }

        // ------------------- INTAKE  -------------------
        Intake.Command intakeCmd = intake.newCommand();

        intakeCmd.autoShoot             = gamepad1.a;
        intakeCmd.autoShootJustPressed  = gamepad1.aWasPressed();
        intakeCmd.autoShootFeedOverride = gamepad1.right_bumper;
        intakeCmd.manualIntakeForward = gamepad2.b;
        intakeCmd.manualIntakeReverse = gamepad2.a;
        intakeCmd.manualIndexerUp     = gamepad2.right_bumper;
        intakeCmd.autoBlockEnabled = (!gamepad1.a || !gamepad1.left_bumper);
        intakeCmd.shooterClearing = gamepad1.left_bumper;

        intake.update(intakeCmd);

        // ------------------- SHOOTER -------------------
        if (gamepad1.left_trigger >= 0.3) {
            robot.shooter.shooterTargetVelocity = 1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

        } else if (gamepad1.left_bumper) {
            robot.shooter.shooterTargetVelocity = -1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

        } else if (gamepad1.right_trigger >= 0.3) {
            robot.shooter.aimingLimelight = true;
            robot.turret.aimingLimelight  = true;
        } else {
            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

            robot.shooter.shooterTargetVelocity = 0;
        }

        // ------------------- TURRET -------------------
        if (gamepad1.y || gamepad2.right_trigger >= 0.3) {
            robot.turret.aimingLimelight = true;
        } else if (gamepad1.right_trigger < 0.3) {
            robot.turret.aimingLimelight = false;
            robot.torrettCoquette.setPower(-gamepad2.left_stick_x);
        }

        // ------------------- LIFT (LIBRO / SUBEBAJA) -------------------
        if (gamepad2.dpad_up) {
            robot.subiBajaMotor.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            robot.subiBajaMotor.setPower(-0.25);
        } else {
            robot.subiBajaMotor.setPower(0.0);
        }

        // ------------------- ROBOT -------------------
        robot.update();

        // ------------------- COLOR SENSOR + RGB INDICATOR -------------------
        int red   = robot.detectaBolas.red();
        int green = robot.detectaBolas.green();
        int blue  = robot.detectaBolas.blue();

        boolean hayBola = intake.hasBall();
        updateRgbIndicator(hayBola, red, green, blue);

        // ------------------- TELEMETRY -------------------
        telemetry.addData("Bolas rojas", red);
        telemetry.addData("Bolas verdes", green);
        telemetry.addData("Bolas azules", blue);
        telemetry.addData("Hay bola", hayBola);
        telemetry.addData("Sube bolas", intake.getIndexerPosition());
        telemetry.addData("Para bolas", intake.getGatePosition());
        telemetry.addData("Ángulo", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.update();
    }

    private void updateRgbIndicator(boolean hayBola, int r, int g, int b) {
        if (robot.light == null) return;

        int sum = r + g + b;
        boolean brightEnough = sum >= COLOR_MIN_BRIGHTNESS;

        boolean isGreen = brightEnough &&
                g > r * COLOR_DOMINANT_RATIO &&
                g > b * COLOR_DOMINANT_RATIO;

        boolean isPurple = brightEnough &&
                r > g * COLOR_DOMINANT_RATIO &&
                b > g * COLOR_DOMINANT_RATIO;

        if (hayBola && isGreen) {
            robot.light.setPosition(RGB_GREEN);
        } else if (hayBola && isPurple) {
            robot.light.setPosition(RGB_VIOLET);
        } else {
            updateRgbSpectrum();
        }
    }

    private void updateRgbSpectrum() {
        double t = rgbCycleTimer.seconds();
        double phase = (t % RGB_CYCLE_PERIOD) / RGB_CYCLE_PERIOD; // 0..1

        // Sweep linearly from red to violet
        double pos = RGB_SPEC_MIN + phase * (RGB_SPEC_MAX - RGB_SPEC_MIN);

        robot.light.setPosition(pos);
    }
}
