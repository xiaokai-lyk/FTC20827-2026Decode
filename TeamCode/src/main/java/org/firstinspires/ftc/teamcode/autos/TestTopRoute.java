// Test.java
package org.firstinspires.ftc.teamcode.autos;

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

/**
 * 改进版自动驾驶测试程序
 * 使用状态机模式提高可扩展性和可维护性
 */
@Autonomous(name = "TestTopRoute", group = "autos")
public class TestTopRoute extends XKCommandOpmode {
    // 硬件子系统

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
    private CommandScheduler scheduler = CommandScheduler.getInstance();

    // 定义自动驾驶步骤枚举
    private enum AutoStep {
        MOVE_TO_FIRST_POSITION,           // 初始化射击器
        FIRST_SHOOT_BALLS,            // 射球阶段
        MOVE_TO_INTAKE_POSITION1,// 第一组球
        INTAKE_BALLS1,
        MOVE_TO_SHOOTING_POSITION1,
        SHOOT_BALLS1,
        MOVE_TO_INTAKE_POSITION2,// 第二组球
        INTAKE_BALLS2,
        MOVE_TO_SHOOTING_POSITION2,
        SHOOT_BALLS2,
        MOVE_TO_INTAKE_POSITION3,// 第三组球
        INTAKE_BALLS3,
        MOVE_TO_SHOOTING_POSITION3,
        SHOOT_BALLS3,
        STOP_SYSTEMS,           // 停止所有系统
        COMPLETE               // 完成
    }

    @Override
    public void onStart() {
        // 初始化状态机
        currentStep = AutoStep.MOVE_TO_FIRST_POSITION;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Status", "Started");
    }

    @Override
    public void run() {
        // 更新传感器数据
        hardwares.sensors.odo.update();
        odo = new OdometerData(hardwares.sensors.odo);

        // 执行当前步骤
        executeCurrentStep();

        // 运行命令调度器
        scheduler.run();

        // 更新遥测数据
        updateTelemetry();
    }

    /**
     * 根据当前步骤执行相应操作
     */
    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_FIRST_POSITION:
                moveToShootingPos(0);
                break;

            case FIRST_SHOOT_BALLS:
                shootBalls();
                break;

            case MOVE_TO_INTAKE_POSITION1:
                moveToIntakePos(0);
                break;

            case INTAKE_BALLS1:
                IntakeBalls(0);
                break;

            case MOVE_TO_SHOOTING_POSITION1:
                moveToShootingPos(0);
                break;

            case SHOOT_BALLS1:
                shootBalls();
                break;

            case MOVE_TO_INTAKE_POSITION2:
                moveToIntakePos(1);
                break;

            case INTAKE_BALLS2:
                IntakeBalls(1);
                break;

            case MOVE_TO_SHOOTING_POSITION2:
                moveToShootingPos(0);
                break;

            case SHOOT_BALLS2:
                shootBalls();
                break;

            case MOVE_TO_INTAKE_POSITION3:
                moveToIntakePos(2);
                break;

            case INTAKE_BALLS3:
                IntakeBalls(2);
                break;

            case MOVE_TO_SHOOTING_POSITION3:
                moveToShootingPos(0);
                break;

            case SHOOT_BALLS3:
                shootBalls();
                break;

            case STOP_SYSTEMS:
                stopSystems();
                break;

            case COMPLETE:
                // 自动程序完成，无需进一步操作
                break;
        }
    }

    /**
     * 处理初始化射击器步骤
     */
    private void moveToShootingPos(int posNum) {
        // 设置射击器和进球系统
        shooter.blockBallPass().schedule();
        shooter.setShooter(Constants.shooter105cm).schedule();
        intake.startIntake(false).schedule();

        // 驱动到第一个位置
        AutoDrive.Output out = autoDrive.driveToAdaptive(
            drive,
            adaptiveController,
            Constants.shootingPosition[posNum][0],  // X坐标
            Constants.shootingPosition[posNum][1],     // Y坐标
            Constants.shootingPosition[posNum][2],     // 角度
            odo,
            0.5,
            true
        );

        // 检查是否到达位置且运行时间超过5秒
        if (out.atPosition && out.atHeading) {
            transitionToNextStep();
        }
    }

    /**
     * 处理射球步骤
     */
    private void shootBalls() {
        // 允许球通过并开始进球
        shooter.allowBallPass().schedule();

        // 持续3秒后进入下一步
        if (getElapsedSeconds() > 2) {
            transitionToNextStep();
        }
    }

    /**
     * 处理移动到第二个位置步骤
     */
    private void moveToIntakePos(int posNum) {
        // 阻止球通过
        intake.stopIntake().schedule();
        shooter.stopPreShooter().schedule();
        shooter.setShooter(Constants.shooterStop).schedule();

        // 驱动到第二个位置
        AutoDrive.Output out = autoDrive.driveToAdaptive(
            drive,
            adaptiveController,
            Constants.pickUpPosition[posNum][0],  // X坐标
            Constants.pickUpPosition[posNum][1],   // Y坐标
            Constants.pickUpPosition[posNum][2],     // 角度
            odo,
            0.5,
            true
        );

        // 检查是否到达位置且运行时间超过3秒
        if (out.atPosition && out.atHeading) {
            transitionToNextStep();
        }
    }

    /**
     * 处理 intake 步骤
     */
    private void IntakeBalls(int posNum) {
        intake.startIntake(true).schedule();
        shooter.blockBallPass().schedule();

        AutoDrive.Output out = autoDrive.driveToAdaptive(
            drive,
            adaptiveController,
            Constants.pickUpPosition[posNum][0],  // X坐标
            Constants.pickUpPosition[posNum][1]+90,   // Y坐标
            Constants.pickUpPosition[posNum][2],     // 角度
            odo,
            0.3,
            true
        );

        if (out.atPosition && out.atHeading) {
            transitionToNextStep();
        }
    }

    /**
     * 处理停止所有系统步骤
     */
    private void stopSystems() {
        // 停止射击器和进球系统
        shooter.setShooter(Constants.shooterStop).schedule();
        intake.stopIntake().schedule();

        // 完成自动程序
        transitionToNextStep(AutoStep.COMPLETE);
    }

    /**
     * 转换到下一个步骤
     * @param nextStep 下一个步骤
     */
    private void transitionToNextStep(AutoStep nextStep) {
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    /**
     * 转换到下一个步骤
     */
    private void transitionToNextStep() {
        currIndex++;
        AutoStep nextStep = AutoStep.values()[currIndex];
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    /**
     * 获取自步骤开始以来经过的秒数
     * @return 经过的秒数
     */
    private double getElapsedSeconds() {
        return (System.currentTimeMillis() - stepStartTime) / 1000.0;
    }

    /**
     * 更新遥测数据显示
     */
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
        // 初始化所有硬件子系统
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = new AdaptivePoseController();
        shooter = new Shooter(hardwares);
        intake = new Intake(hardwares);
        odo = new OdometerData(hardwares.sensors.odo);
        hardwares.sensors.odo.setHeading(45, AngleUnit.DEGREES);
        telemetry.addData("Auto Status", "Initialized");
    }
}
