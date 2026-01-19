package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoPan;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.OdoData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

/**
 * AutoPan 测试 TeleOp
 *
 * 控制说明：
 * - Gamepad 1 A: 切换 HOLD / TRACK 模式
 * - Gamepad 1 B: 重置目标坐标到 (0, 0)
 * - Gamepad 1 X/Y: 调整测试目标点 X 坐标 (+/- 10cm)
 * - Gamepad 1 Dpad Up/Down: 调整测试目标点 Y 坐标 (+/- 10cm)
 */
@Config
@TeleOp(name = "AutoPan Test", group = "Tests")
public class PanTeleOp extends XKCommandOpmode {
    public static double panMotorPPos = 1;
    public static double panMotorPVel = 1;
    public static double panMotorI = 0;
    public static double panMotorD = 0;
    public static double panMotorF = 0;

    private Hardwares hardwares;
    private AutoPan autoPan;

    // 按键消抖计时器
    private final ElapsedTime buttonTimer = new ElapsedTime();
    private static final double BUTTON_DELAY = 0.3; // 300ms 延迟

    private Drive drive;
    private Drive.DriveCommand driveCommand;
    private FtcDashboard dashboard;


    @Override
    public void initialize() {
        // 1. 初始化硬件映射
        hardwares = new Hardwares(hardwareMap);

        // 2. 初始化 AutoPan，设置初始目标为 (0,0)
        // 如果需要测试其他目标点，请在此处修改坐标，例如 (100, 100)
        autoPan = new AutoPan(hardwares, 100.0, 0.0);
        autoPan.init();

        // 3. 初始化底盘
        drive = new Drive(hardwares);
        driveCommand = new Drive.DriveCommand(
                drive,
                () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x,
                () -> new OdoData(hardwares.sensors.odo),
                1,
                true,
                false
        );

        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized. Press START.");
    }

    @Override
    public void onStart() {
        // 3. 启动 AutoPan (电机归零并使能)
        autoPan.setup();
        buttonTimer.reset();
    }

    @Override
    public void run() {
        // ---底盘控制---
        driveCommand.execute();


        // --- 控制逻辑 ---

        // [A] 切换模式
        if (gamepad1.a && buttonTimer.seconds() > BUTTON_DELAY) {
            autoPan.setMode();
            buttonTimer.reset();
        }


        // 4. 获取最新的里程计数据
        // 注意：OdoData 的获取方式取决于你的项目结构，这里假设 Hardwares 里有 OdoData
        // 或者你需要手动从 Sensors 读取并封装。
        // 根据 OdoData.java 的定义：
        OdoData odoData = new OdoData(hardwares.sensors.odo);

        // 5. 运行 AutoPan 更新
        autoPan.run(odoData);
        hardwares.sensors.odo.update();
        // --- 遥测显示 ---

        // 显示 AutoPan 的详细状态
        AutoPan.TelemetryState panState = autoPan.getTelemetryStatus();
        telemetry.addData("=== Auto Pan Status ===", "");
        telemetry.addData("Mode", panState.mode);
        telemetry.addData("Target (X, Y)", "(%.1f, %.1f)", panState.targetX, panState.targetY);
        telemetry.addData("Angles (Current / Target)", "%.1f / %.1f", panState.currentAngle, panState.filteredAngle);
        telemetry.addData("Limit Reached", panState.isLimitReached);
        telemetry.addData("Motor Power", "%.2f", panState.motorPower);

        // 显示机器人的当前位置以供对参考
        telemetry.addData("=== Robot Pose ===", "");
        telemetry.addData("X", "%.1f cm", odoData.getRobotX());
        telemetry.addData("Y", "%.1f cm", odoData.getRobotY());
        telemetry.addData("Heading", "%.1f deg", odoData.getHeadingDegrees());

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("panMotor/currPos", hardwares.motors.pan.getCurrentPosition());
        packet.put("panMotor/targetPos", hardwares.motors.pan.getTargetPosition());
        dashboard.sendTelemetryPacket(packet);

        autoPan.setPanMotorPIDF(panMotorPPos, panMotorPVel, panMotorI, panMotorD, panMotorF);

        dashboard.getTelemetry().update();
    }
}
