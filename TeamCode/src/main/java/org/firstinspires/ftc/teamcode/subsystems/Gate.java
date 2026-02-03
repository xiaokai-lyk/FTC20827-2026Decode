package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.Hardwares;

/**
 * 闸门系统
 * 
 * 功能：
 * 1. 控制闸门开合
 * 2. 获取当前闸门角度
 */
public class Gate {
    
    // ==========================================
    // 常量配置
    // ==========================================
    private static final double GATE_OPEN_ANGLE = 145;    // 闸门开启角度
    private static final double GATE_CLOSE_ANGLE = 67;  // 闸门关闭角度
    
    // ==========================================
    // 成员变量
    // ==========================================
    private ServoEx gate;

    /**
     * 构造闸门系统实例
     * @param hardwares 硬件映射
     */
    public Gate(@NonNull Hardwares hardwares){
        this.gate = hardwares.servos.gate;
    }

    /**
     * 获取开启闸门的命令
     * @return 开启闸门的InstantCommand
     */
    public InstantCommand open(){
        return new InstantCommand(
                () -> gate.turnToAngle(GATE_OPEN_ANGLE)
        );
    }
    

    /**
     * 获取关闭闸门的命令
     * @return 关闭闸门的InstantCommand
     */
    public InstantCommand close(){
        return new InstantCommand(
                () -> gate.turnToAngle(GATE_CLOSE_ANGLE)
        );
    }

    /**
     * 获取当前闸门的角度
     * @return 当前闸门角度（度）
     */
    public double getAngleDeg(){
        return gate.getAngle();
    }
}
