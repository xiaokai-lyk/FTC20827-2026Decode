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

@Autonomous(name = "TestBottomRoute", group = "autos")
public class TestBottomRoute extends XKCommandOpmode
{
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

    int distanceType = 0; //0是远射 1是中射 2是近射

    /**
     * 定义自动驾驶步骤枚举，表示机器人在自动阶段中的各个任务节点
     */
    private enum AutoStep {
        MOVE_TO_SHOOTING_POSITION,        //初始射球预热
        INITIAL_POSITION_SHOOT,           //初始位置射击
        MOVE_TO_INTAKE_POSITION1,         // 移动至第一组取球点
        INTAKE_BALLS1,                    // 取第一组球
        MOVE_TO_SHOOTING_POSITION1,       // 回到射击位1
        FAR_POSITION_SHOOT,               // 发射第一组球
        MOVE_TO_INTAKE_POSITION2,         // 移动至第二组取球点
        INTAKE_BALLS2,                    // 取第二组球
        MOVE_TO_SHOOTING_POSITION2,       // 回到射击位2
        SHOOT_BALLS2,                     // 发射第二组球
        AWAY_FROM_LINE,                   // 离线
        STOP_SYSTEMS,                     // 停止所有系统
        COMPLETE                          // 完成整个流程
    }

    /**
     * 当OpMode启动时调用此方法，用于初始化状态机变量
     */
    @Override
    public void onStart() {
        // 初始化状态机
        currentStep = AutoStep.MOVE_TO_SHOOTING_POSITION;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Status", "Started");
    }

    /**
     * 主循环中持续运行的方法，负责更新传感器、执行当前步骤和调度命令
     */
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
     * 根据当前步骤执行相应的操作逻辑
     */
    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_SHOOTING_POSITION:
                moveToShootingPos(2);//远射位置
                break;

            case INITIAL_POSITION_SHOOT:
                shootBalls();
                break;

            case MOVE_TO_INTAKE_POSITION1:
                moveToIntakePos(2);
                break;

            case INTAKE_BALLS1:
                IntakeBalls(0);
                break;

            case MOVE_TO_SHOOTING_POSITION1:
                moveToShootingPos(2);//远射位置
                break;


            case FAR_POSITION_SHOOT:
                shootBalls();
                break;

            case MOVE_TO_INTAKE_POSITION2:
                moveToIntakePos(1);
                break;

            case INTAKE_BALLS2:
                IntakeBalls(1);
                break;

            case MOVE_TO_SHOOTING_POSITION2:
                distanceType = 1; //将移动期间的预热改为中射预热
                moveToShootingPos(1);//中射位置
                break;

            case SHOOT_BALLS2:
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
                // 自动程序完成，无需进一步操作
        }
    }

    /**
     * 控制机器人移动到指定编号的射击位置，并设置射击准备动作
     *
     * @param posNum 射击位置索引（对应Constants.shootingPosition数组）
     */
    private void moveToShootingPos(int posNum) {
        // 设置射击器和进球系统
        if(distanceType == 0)
        {
            shooter.blockBallPass().schedule();
            shooter.setShooter(Constants.shooter250cm).schedule();
            intake.startIntake(false).schedule();
        }
        else
        {
            shooter.blockBallPass().schedule();
            shooter.setShooter(Constants.shooter105cm).schedule();
            intake.startIntake(false).schedule();
        }


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
     * 执行发射球的动作，在允许球通过后等待一段时间再进入下一阶段
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
     * 控制机器人前往指定编号的取球点，并关闭预处理机构
     *
     * @param posNum 取球位置索引（对应Constants.pickUpPosition数组）
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
     * 在当前位置进行取球动作，控制驱动向前进以确保拾取成功
     *
     * @param posNum 取球位置索引（对应Constants.pickUpPosition数组）
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
     * 停止所有系统的运作，包括射击与取球装置，并结束自动流程
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
     *
     * @param nextStep 下一个要切换的状态
     */
    private void transitionToNextStep(@NonNull AutoStep nextStep) {
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    private void moveFromLine() {
        AutoDrive.Output out = autoDrive.driveToAdaptive(
            drive,
            adaptiveController,
            Constants.pickUpPosition[2][0],  // X坐标
            Constants.pickUpPosition[2][1],   // Y坐标
            Constants.pickUpPosition[2][2],     // 角度
            odo,
            0.3,
            true
        );
    }

    /**
     * 转换到下一个步骤（基于枚举顺序递增）
     */
    private void transitionToNextStep() {
        currIndex++;
        AutoStep nextStep = AutoStep.values()[currIndex];
        currentStep = nextStep;
        stepStartTime = System.currentTimeMillis();
        telemetry.addData("Auto Step Changed", nextStep.toString());
    }

    /**
     * 计算从当前步骤开始至今所经历的时间（单位：秒）
     *
     * @return 已经过去的时间（秒）
     */
    private double getElapsedSeconds() {
        return (System.currentTimeMillis() - stepStartTime) / 1000.0;
    }

    /**
     * 实时更新遥测信息显示当前状态及机器人的实时定位数据
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

    /**
     * 初始化所有硬件组件及其对应的子系统对象
     */
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
