package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

@Autonomous(name = "BottomRouteBlue", group = "autos")
public class BottomRouteBlue extends XKCommandOpmode {
    private Hardwares hardwares;
    private Drive drive;
    private AutoDrive autoDrive;
    private AdaptivePoseController adaptiveController;
    private Shooter shooter;
    private Intake intake;

    // 状态机相关
    private AutoStep currentStep;
    private int currIndex;
    private long stepStartTime;
    private OdometerData odo;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    int rounds = 1;

    /**
     * 定义自动驾驶步骤枚举，表示机器人在自动阶段中的各个任务节点
     */
    private enum AutoStep {
        WAIT_FOR_ACCELERATION,
        INITIAL_POSITION_SHOOT,
        AWAY_FROM_LINE,
        STOP_SYSTEMS,
        COMPLETE
    }

    /**
     * 当OpMode启动时调用此方法，用于初始化状态机变量
     */
    @Override
    public void onStart() {
        currentStep = AutoStep.WAIT_FOR_ACCELERATION;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Status", "Started");
    }

    /**
     * 主循环中持续运行的方法，负责更新传感器、执行当前步骤和调度命令
     */
    @Override
    public void run() {
        hardwares.sensors.odo.update();
        odo = new OdometerData(hardwares.sensors.odo);

        executeCurrentStep();
        scheduler.run();
        updateTelemetry();
    }

    /**
     * 根据当前步骤执行相应的操作逻辑
     */
    private void executeCurrentStep() {
        switch (currentStep) {
            case WAIT_FOR_ACCELERATION:
                waitForAcceleration();
                break;

            case INITIAL_POSITION_SHOOT:
                shootBalls();
                break;

            case AWAY_FROM_LINE:
                moveFromLine();
                break;

            case STOP_SYSTEMS:
                stopSystems();
                break;

            case COMPLETE:
                break;
        }
    }

    /**
     * 等待射击系统加速完成
     */
    public void waitForAcceleration() {
        shooter.setShooter(Constants.shooterFar).schedule();
        if (getElapsedSeconds() > 4) {
            transitionToNextStep();
        }
    }

    /**
     * 执行发射球的动作，在允许球通过后等待一段时间再进入下一阶段
     */
    private void shootBalls() {
        double timeEachLoop = getElapsedSeconds() % 3;

        if (rounds != 3 && timeEachLoop > 2.8 && shooter.shooterReady(Constants.shooterFar)) {
            intake.startIntake().schedule();
            shooter.allowBallPassClose().schedule();
            rounds++;
        } else if (rounds == 3 && timeEachLoop > 2.7 && shooter.shooterReady(Constants.shooterFar)) {
            intake.startIntake().schedule();
            shooter.allowBallPassClose().schedule();
        } else {
            intake.stopIntake().schedule();
            shooter.blockBallPass().schedule();
        }

        if (getElapsedSeconds() > 10) {
            transitionToNextStep();
        }
    }

    /**
     * 控制机器人前往指定编号的取球点，并关闭预处理机构
     */
    private void moveToIntakePos(int posNum) {
        intake.stopIntake().schedule();
        shooter.stopPreShooter().schedule();

        AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                Constants.bluePickUpPosition[posNum].x,
                Constants.bluePickUpPosition[posNum].y,
                Constants.bluePickUpPosition[posNum].heading,
                odo,
                0.65,
                true
        );

        if (out.atPosition && out.atHeading || getElapsedSeconds() > 5) {
            transitionToNextStep();
        }
    }

    /**
     * 在当前位置进行取球动作，控制驱动向前进以确保拾取成功
     */
    private void IntakeBalls(int posNum) {
        intake.startIntake().schedule();
        shooter.blockBallPass().schedule();

        AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                Constants.blueBallPosition[posNum].x,
                Constants.blueBallPosition[posNum].y,
                Constants.blueBallPosition[posNum].heading,
                odo,
                0.7,
                true
        );

        if ((out.atPosition && out.atHeading) || getElapsedSeconds() > 2) {
            transitionToNextStep();
        }
    }

    /**
     * 停止所有系统的运作，包括射击与取球装置，并结束自动流程
     */
    private void stopSystems() {
        shooter.setShooter(Constants.shooterStop).schedule();
        intake.stopIntake().schedule();
        transitionToNextStep(AutoStep.COMPLETE);
    }

    private void moveFromLine() {
        // 注意：此处使用了硬编码坐标。若你有 Constants.blueParkPositionBottom，
        // 建议替换为：
        // Constants.blueParkPositionBottom.x, .y, .heading
        AutoDrive.Output out = autoDrive.driveToAdaptive(
                drive,
                adaptiveController,
                5,   // X
                45,  // Y
                0,   // heading
                odo,
                1,
                false
        );
        if (getElapsedSeconds() > 3) {
            transitionToNextStep();
        }
    }

    private void transitionToNextStep(@NonNull AutoStep nextStep) {
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    private void transitionToNextStep() {
        currIndex++;
        AutoStep nextStep = AutoStep.values()[currIndex];
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    private double getElapsedSeconds() {
        return (System.currentTimeMillis() - stepStartTime) / 1000.0;
    }

    private void updateTelemetry() {
        telemetry.addData("Current Step", currentStep.toString());
        telemetry.addData("Step Elapsed Time", "%.1f sec", getElapsedSeconds());
        if (odo != null) {
            telemetry.addData("Robot Position", "X:%.1f Y:%.1f H:%.1f",
                    odo.getRobotPosition().getX(DistanceUnit.MM),
                    odo.getRobotPosition().getY(DistanceUnit.MM),
                    Math.toDegrees(odo.getHeadingRadians()));
        }
        telemetry.update();
    }

    @Override
    public void initialize() {
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = Constants.PID.newPoseController();
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        odo = new OdometerData(hardwares.sensors.odo);
        hardwares.sensors.odo.setHeading(13.5, AngleUnit.DEGREES);
        telemetry.addData("Auto Status", "Initialized");
    }
}
