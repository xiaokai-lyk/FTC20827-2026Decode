package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class XKHardware<T> {
    private final T hardware;

    public XKHardware(T hardware) {
        this.hardware = hardware;
    }

    public void logName(String name) {
        System.out.println("Hardware name: " + name + ", type: " + hardware.getClass().getSimpleName());
    }

    public T device() {
        return hardware;
    }

    // 简洁的静态工厂方法
    public static <T> XKHardware<T> of(HardwareMap hardwareMap, String name, Class<T> type) {
        T hw = hardwareMap.get(type, name);
        return new XKHardware<>(hw);
    }
}
