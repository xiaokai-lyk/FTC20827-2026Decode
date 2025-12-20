package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardwares;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils.AdaptivePIDController;
import org.firstinspires.ftc.teamcode.utils.AdaptivePoseController;
import org.firstinspires.ftc.teamcode.utils.OdometerData;
import org.firstinspires.ftc.teamcode.utils.XKCommandOpmode;

import java.util.Random;

/**
 * PID 调参 + 模拟退火自动搜索
 * 按键说明:
 *  A: 选择 前后 循环测试 (xTrans 平移)
 *  B: 选择 左右 循环测试 (yTrans 平移)
 *  X: 选择 旋转 循环测试 (rotate)
 *  Y: 切换是否启动当前模式的模拟退火自动调参 (再次按关闭)
 *  BACK: 复位状态机与 PID
 *
 * 循环测试: 在 起点 与 目标 之间往返, 每次记录 从开始移动 到 atPosition && atHeading 的用时
 * 模拟退火: 在选定模式下自动迭代 PID 参数, 目标是最小化到达用时 (cost = arrivalTimeSec)
 */
@Disabled
@Autonomous(name = "PidAnnealTest", group = "autos")
public class PidAnnealTest extends XKCommandOpmode {
    // 基础硬件
    private Hardwares hardwares;
    private Drive drive;
    private AutoDrive autoDrive;

    // 当前使用的姿态控制器 (会被重新创建以更新 PID 参数)
    private AdaptivePoseController adaptiveController;

    // 起点记录
    private double startXcm, startYcm, startHeadingDeg;

    // 目标偏移/角度
    private double deltaForwardCm = 120; // 前后测试位移 (改变 targetY)
    private double deltaSideCm = 120;    // 左右测试位移 (改变 targetX)
    private double deltaRotateDeg = 180; // 旋转测试角度

    // 循环测试状态
    private boolean goingToTarget = true; // true 去目标, false 回起点
    private long legStartNano = 0;        // 本轮开始时间戳
    private boolean legStarted = false;   // 已经检测到运动开始
    private boolean legCompleted = false; // 已经记录结束
    private double lastArrivalSec = 0;    // 最近一次到达耗时
    private int stableCount = 0;          // 到位稳定计数
    private static final int STABLE_MIN = 5;
    // 新增: 最近3次耗时统计
    private final double[] arrivalTimes = new double[3];
    private int arrivalIndex = 0;
    private int arrivalCount = 0; // 已记录次数 (<=3)
    private double arrivalAvgSec = 0.0;

    // 运行模式
    private enum Mode { NONE, FORWARD_BACK, LEFT_RIGHT, ROTATE }
    private Mode currentMode = Mode.NONE;

    // 模拟退火状态
    private boolean annealActive = false; // 是否正在自动调参
    private double temperature = 1.0;     // 当前温度
    private double coolingRate = 0.92;    // 降温率
    private double minTemperature = 0.02; // 最低温度终止
    private int iteration = 0;            // 当前迭代次数
    private int maxIterations = 200;      // 最大迭代次数
    private double maxCandidateTimeSec = 6.0; // 单次候选测试最大允许时间 (超时视为失败)

    // 当前接受的参数 & 成本
    private double currKp, currKi, currKd;
    private double currCost = Double.POSITIVE_INFINITY;

    // 最佳参数 & 成本
    private double bestKp, bestKi, bestKd;
    private double bestCost = Double.POSITIVE_INFINITY;

    // 当前候选参数 (准备评估)
    private double candKp, candKi, candKd;
    private boolean candidateRunning = false; // 正在跑一次测试以评估候选
    private enum CandidatePhase { OUTBOUND, INBOUND }
    private CandidatePhase candidatePhase = CandidatePhase.OUTBOUND;

    private final Random rng = new Random();

    // PID 参数范围 (按模式区分)
    private static class Bounds { double kpMin, kpMax, kiMin, kiMax, kdMin, kdMax; }
    private final Bounds translationalBounds = new Bounds();
    private final Bounds rotationalBounds = new Bounds();

    // 为了检测按键边缘
    private boolean prevA, prevB, prevX, prevY, prevBack;

    @Override
    public void initialize() {
        hardwares = new Hardwares(hardwareMap);
        drive = new Drive(hardwares);
        autoDrive = new AutoDrive();
        adaptiveController = Constants.PID.newPoseController(); // 初始用默认

        // 捕获起点
        hardwares.sensors.odo.update();
        startXcm = hardwares.sensors.odo.getPosX(DistanceUnit.CM);
        startYcm = hardwares.sensors.odo.getPosY(DistanceUnit.CM);
        startHeadingDeg = hardwares.sensors.odo.getHeading(AngleUnit.DEGREES);

        // 设定参数范围
        translationalBounds.kpMin = 0.01; translationalBounds.kpMax = 0.50;
        translationalBounds.kiMin = 0.00; translationalBounds.kiMax = 0.10;
        translationalBounds.kdMin = 0.00; translationalBounds.kdMax = 0.25;
        rotationalBounds.kpMin = 0.20; rotationalBounds.kpMax = 2.0;
        rotationalBounds.kiMin = 0.00; rotationalBounds.kiMax = 0.10;
        rotationalBounds.kdMin = 0.00; rotationalBounds.kdMax = 0.50;

        // 初始化当前参数为常量初始值 (根据模式之后会覆写)
        currKp = Constants.PID.TRANS_KP;
        currKi = Constants.PID.TRANS_KI;
        currKd = Constants.PID.TRANS_KD;
        bestKp = currKp; bestKi = currKi; bestKd = currKd;
    }

    @Override
    public void onStart() {
        resetLegState();
    }

    private void resetLegState(){
        goingToTarget = true;
        legStartNano = 0;
        legStarted = false;
        legCompleted = false;
        lastArrivalSec = 0;
        stableCount = 0;
        if (adaptiveController != null) adaptiveController.reset();
    }

    private void edgeProcess(Gamepad gp){
        boolean a = gp.a;
        boolean b = gp.b;
        boolean x = gp.x;
        boolean back = gp.back;
        // 模式选择后自动启动退火
        if (a && !prevA){ currentMode = Mode.FORWARD_BACK; startAutoAnneal(); }
        if (b && !prevB){ currentMode = Mode.LEFT_RIGHT; startAutoAnneal(); }
        if (x && !prevX){ currentMode = Mode.ROTATE; startAutoAnneal(); }
        if (back && !prevBack){ resetAnneal(); resetLegState(); annealActive = true; }
        prevA = a; prevB = b; prevX = x; prevBack = back;
    }

    private void startAutoAnneal(){ annealActive = true; resetAnneal(); resetLegState(); prepareInitialPIDForMode(); installCurrentPID(); candidateRunning = false; }

    private void resetAnneal(){
        temperature = 1.0;
        iteration = 0;
        currCost = Double.POSITIVE_INFINITY;
        bestCost = Double.POSITIVE_INFINITY;
        candidateRunning = false;
    }

    private void prepareInitialPIDForMode(){
        switch (currentMode){
            case FORWARD_BACK:
            case LEFT_RIGHT:
                currKp = Constants.PID.TRANS_KP;
                currKi = Constants.PID.TRANS_KI;
                currKd = Constants.PID.TRANS_KD;
                break;
            case ROTATE:
                currKp = Constants.PID.ROT_KP;
                currKi = Constants.PID.ROT_KI;
                currKd = Constants.PID.ROT_KD;
                break;
            default:
                break;
        }
        bestKp = currKp; bestKi = currKi; bestKd = currKd;
    }

    private AdaptivePIDController makeXTrans(double kp,double ki,double kd){
        return new AdaptivePIDController(kp, ki, kd,
                Constants.PID.TRANS_MAX_OUT,
                Constants.PID.TRANS_MIN_CMD,
                Constants.PID.TRANS_DEADZONE_CM,
                Constants.PID.TRANS_I_CLAMP);
    }
    private AdaptivePIDController makeYTrans(double kp,double ki,double kd){
        return new AdaptivePIDController(kp, ki, kd,
                Constants.PID.TRANS_MAX_OUT,
                Constants.PID.TRANS_MIN_CMD,
                Constants.PID.TRANS_DEADZONE_CM,
                Constants.PID.TRANS_I_CLAMP);
    }
    private AdaptivePIDController makeRotate(double kp,double ki,double kd){
        return new AdaptivePIDController(kp, ki, kd,
                Constants.PID.ROT_MAX_OUT,
                Constants.PID.ROT_MIN_CMD,
                Constants.PID.ROT_DEADZONE_RAD,
                Constants.PID.ROT_I_CLAMP);
    }

    private void installCurrentPID(){
        AdaptivePIDController x = Constants.PID.xTrans();
        AdaptivePIDController y = Constants.PID.yTrans();
        AdaptivePIDController r = Constants.PID.rotate();
        switch (currentMode){
            case FORWARD_BACK:
                x = makeXTrans(currKp, currKi, currKd);
                break;
            case LEFT_RIGHT:
                y = makeYTrans(currKp, currKi, currKd);
                break;
            case ROTATE:
                r = makeRotate(currKp, currKi, currKd);
                break;
        }
        adaptiveController = new AdaptivePoseController(x, y, r);
    }

    private void installCandidatePID(){
        AdaptivePIDController x = Constants.PID.xTrans();
        AdaptivePIDController y = Constants.PID.yTrans();
        AdaptivePIDController r = Constants.PID.rotate();
        switch (currentMode){
            case FORWARD_BACK:
                x = makeXTrans(candKp, candKi, candKd); break;
            case LEFT_RIGHT:
                y = makeYTrans(candKp, candKi, candKd); break;
            case ROTATE:
                r = makeRotate(candKp, candKi, candKd); break;
        }
        adaptiveController = new AdaptivePoseController(x, y, r);
    }

    private void proposeCandidate(){
        Bounds b = (currentMode == Mode.ROTATE) ? rotationalBounds : translationalBounds;
        double spanKp = b.kpMax - b.kpMin;
        double spanKi = b.kiMax - b.kiMin;
        double spanKd = b.kdMax - b.kdMin;
        // 使用当前参数为中心做随机微扰 (均匀 -1..1 * 温度 * 0.5 * span)
        candKp = clamp(currKp + (rng.nextDouble()*2 -1)* temperature * 0.5 * spanKp, b.kpMin, b.kpMax);
        candKi = clamp(currKi + (rng.nextDouble()*2 -1)* temperature * 0.5 * spanKi, b.kiMin, b.kiMax);
        candKd = clamp(currKd + (rng.nextDouble()*2 -1)* temperature * 0.5 * spanKd, b.kdMin, b.kdMax);
    }

    private double clamp(double v,double lo,double hi){ return v < lo ? lo : (v > hi ? hi : v); }

    @Override
    public void run(){
        hardwares.sensors.odo.update();
        OdometerData odom = new OdometerData(hardwares.sensors.odo);
        edgeProcess(gamepad1);
        if (currentMode == Mode.NONE){ telemetry.addLine("选择模式: A前后 B左右 X旋转 (自动往返退火)"); telemetry.update(); return; }
        annealLoop(odom); telemetry.update();
    }

    private void annealLoop(OdometerData odom){
        if (!annealActive){ return; }
        if (iteration >= maxIterations || temperature < minTemperature){ annealActive = false; telemetry.addLine("退火结束"); return; }
        // 启动新候选 (往返双程)
        if (!candidateRunning){
            proposeCandidate();
            installCandidatePID();
            goingToTarget = true; // 去目标
            candidatePhase = CandidatePhase.OUTBOUND;
            legStartNano = 0;
            legStarted = false;
            stableCount = 0;
            candidateRunning = true;
        }
        // 根据候选阶段确定当前目标
        double goalX = goingToTarget ? startXcm : startXcm;
        double goalY = goingToTarget ? startYcm : startYcm;
        double goalHeadingDeg = goingToTarget ? startHeadingDeg : startHeadingDeg;
        switch (currentMode){
            case FORWARD_BACK:
                goalY = (candidatePhase == CandidatePhase.OUTBOUND) ? startYcm + deltaForwardCm : startYcm;
                break;
            case LEFT_RIGHT:
                goalX = (candidatePhase == CandidatePhase.OUTBOUND) ? startXcm + deltaSideCm : startXcm;
                break;
            case ROTATE:
                goalHeadingDeg = (candidatePhase == CandidatePhase.OUTBOUND) ? startHeadingDeg + deltaRotateDeg : startHeadingDeg;
                break;
        }
        AutoDrive.Output out = autoDrive.driveToAdaptive(drive, adaptiveController, goalX, goalY, goalHeadingDeg, odom, 1.0, true);
        // 记录开始时间
        if (!legStarted && (!out.atPosition || !out.atHeading)){
            legStarted = true;
            legStartNano = System.nanoTime();
        }
        // 稳定判定
        if (out.atPosition && out.atHeading){ stableCount++; } else { stableCount = 0; }
        double elapsedSec = legStarted ? (System.nanoTime() - legStartNano)/1e9 : 0.0;
        boolean timeout = legStarted && elapsedSec > maxCandidateTimeSec;
        // OUTBOUND 完成切换到 INBOUND
        if (candidatePhase == CandidatePhase.OUTBOUND && stableCount >= STABLE_MIN){
            candidatePhase = CandidatePhase.INBOUND;
            goingToTarget = false; // 返程
            stableCount = 0;
            adaptiveController.reset(); // ��� PID 状态，避免积分影响返程
            // 强制重新启动检测
            legStarted = true; // 继续累计总时间 (不重置起始时间) 以测总往返耗时
        }
        // INBOUND 完成则评估成本
        if (candidatePhase == CandidatePhase.INBOUND && stableCount >= STABLE_MIN){
            double roundTripSec = elapsedSec; // 总往返耗时
            lastArrivalSec = roundTripSec;
            arrivalTimes[arrivalIndex] = roundTripSec;
            arrivalIndex = (arrivalIndex + 1) % arrivalTimes.length;
            if (arrivalCount < arrivalTimes.length) arrivalCount++;
            double sum = 0.0; for (int i=0;i<arrivalCount;i++) sum += arrivalTimes[i];
            arrivalAvgSec = sum / arrivalCount;
            double cost = roundTripSec; // 成本定义: 总往返时间
            boolean accept;
            if (currCost == Double.POSITIVE_INFINITY) accept = true;
            else if (cost < currCost) accept = true;
            else accept = rng.nextDouble() < Math.exp(-(cost - currCost)/Math.max(1e-9, temperature));
            if (accept){ currKp = candKp; currKi = candKi; currKd = candKd; currCost = cost; installCurrentPID(); }
            if (cost < bestCost){ bestCost = cost; bestKp = candKp; bestKi = candKi; bestKd = candKd; }
            temperature *= coolingRate;
            iteration++;
            candidateRunning = false;
            stableCount = 0;
        }
        // 超时处理: 直接惩罚并结束候选
        if (timeout && candidateRunning){
            double cost = elapsedSec * 2.0; // 惩罚
            boolean accept = currCost == Double.POSITIVE_INFINITY; // 仅第一轮允许接受
            if (accept){ currKp = candKp; currKi = candKi; currKd = candKd; currCost = cost; installCurrentPID(); }
            if (cost < bestCost){ bestCost = cost; bestKp = candKp; bestKi = candKi; bestKd = candKd; }
            temperature *= coolingRate;
            iteration++;
            candidateRunning = false;
            telemetry.addLine("[超时] 候选失败");
        }
        // Telemetry
        telemetry.addLine("=== 自动往返退火 ===");
        telemetry.addData("模式", currentMode);
        telemetry.addData("阶段", candidatePhase);
        telemetry.addData("迭代", iteration + "/" + maxIterations);
        telemetry.addData("温度", temperature);
        telemetry.addData("当前成本", currCost);
        telemetry.addData("最佳成本", bestCost);
        telemetry.addData("候选总时间(s)", elapsedSec);
        telemetry.addData("稳定计数", stableCount + "/" + STABLE_MIN);
        telemetry.addData("cmd X",out.xCmd);
        telemetry.addData("cmd Y",out.yCmd);
        telemetry.addData("atPos", out.atPosition);
        telemetry.addData("atHead", out.atHeading);
        telemetry.addData("dx", out.dxCm);
        telemetry.addData("dy", out.dyCm);
        telemetry.addData("dTheta(deg)", Math.toDegrees(out.dThetaRad));
        telemetry.addLine("-- 当前PID --"); telemetry.addData("currKp", currKp); telemetry.addData("currKi", currKi); telemetry.addData("currKd", currKd);
        telemetry.addLine("-- 候选PID --"); telemetry.addData("candKp", candKp); telemetry.addData("candKi", candKi); telemetry.addData("candKd", candKd);
        telemetry.addLine("-- 最佳PID --"); telemetry.addData("bestKp", bestKp); telemetry.addData("bestKi", bestKi); telemetry.addData("bestKd", bestKd);
        telemetry.addData("最近往返耗时(s)", lastArrivalSec);
        telemetry.addData("最近3次平均(s)", arrivalAvgSec);
        if (arrivalCount > 0){
            StringBuilder sb = new StringBuilder();
            for (int i=0;i<arrivalCount;i++){ sb.append(String.format("%.2f", arrivalTimes[i])); if(i<arrivalCount-1) sb.append(","); }
            telemetry.addData("往返列表", sb.toString());
        }
    }
}
