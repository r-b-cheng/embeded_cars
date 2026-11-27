package org.embeddedcars.patrol;

final class Frame {
    static final class Ray {
        long stamp;
        int mm, x, y, angle;
    }
    final Ray[] rays = new Ray[3];
    int state, x, y, angle;

    boolean full;
    long boardMs, imuMs, rightPulses, rightTravel, centerTravel, uartErrors, resets, badRequests, maxTick;
    long eventId, eventMs, traceId, traceMs, skipped;
    int rightSpeed, leftRpm10, rightRpm10, centerSpeed, pwmL, pwmR, target, servo, fault;
    int eventCode, traceKind, traceA, traceB;
    int[] imu = new int[6], bearing = new int[3];
    String applied;
    static final String[] STATES = {"待命","行驶","障碍停车","回波无效","指令超时","SW1停车","目标完成","反馈异常","驱动线异常"};
    static final String[] FAULTS = {"正常","IMU未就绪","左轮无脉冲","右轮无脉冲","任务超时或无反馈","转向相反","停车电平异常"};
    boolean extended, imuReady, logFailed;
    int speed, taskId;
    long pulses, travel;

    static Frame parse(String line, int seq) {
        String[] t = line.split(" ", -1);
        if ((t.length != 21 && t.length != 37 && t.length != 64) || !t[0].equals("T") || Integer.parseInt(t[1]) != seq)
            throw new IllegalArgumentException("响应序号或格式错误");
        Frame f = new Frame();
        f.state = Integer.parseInt(t[2]);
        f.x = Integer.parseInt(t[3]);
        f.y = Integer.parseInt(t[4]);
        f.angle = Integer.parseInt(t[5]);
        f.full = t.length == 64;
        f.extended = t.length >= 37;
        if (f.extended) {
            f.boardMs = Long.parseLong(t[21]);
            f.pulses = Long.parseLong(t[22]);
            f.travel = Long.parseLong(t[23]);
            f.speed = Integer.parseInt(t[24]);
            f.imuReady = Integer.parseInt(t[25]) == 1;
            f.taskId = Integer.parseInt(t[26]);
            for (int i = 0; i < 6; i++) f.imu[i] = Integer.parseInt(t[27+i]);
            f.eventId = Long.parseLong(t[33]);
            f.eventMs = Long.parseLong(t[34]);
            f.eventCode = Integer.parseInt(t[35]);
            f.logFailed = Integer.parseInt(t[36]) != 0;
        }
        if (f.full) {
            f.rightPulses = Long.parseLong(t[37]);
            f.rightTravel = Long.parseLong(t[38]);
            f.rightSpeed = Integer.parseInt(t[39]);
            f.leftRpm10 = Integer.parseInt(t[40]);
            f.rightRpm10 = Integer.parseInt(t[41]);
            f.centerSpeed = Integer.parseInt(t[42]);
            f.centerTravel = Long.parseLong(t[43]);
            f.pwmL = Integer.parseInt(t[44]);
            f.pwmR = Integer.parseInt(t[45]);
            f.applied = t[46];
            f.target = Integer.parseInt(t[47]);
            f.imuMs = Long.parseLong(t[48]);
            for (int i = 0; i < 3; i++) f.bearing[i] = Integer.parseInt(t[49+i]);
            f.servo = Integer.parseInt(t[52]);
            f.fault = Integer.parseInt(t[53]);
            if (f.fault < 0 || f.fault >= FAULTS.length) throw new IllegalArgumentException("故障码");
            f.uartErrors = Long.parseLong(t[54]);
            f.resets = Long.parseLong(t[55]);
            f.badRequests = Long.parseLong(t[56]);
            f.maxTick = Long.parseLong(t[57]);
            f.traceId = Long.parseLong(t[58]);
            f.traceMs = Long.parseLong(t[59]);
            f.traceKind = Integer.parseInt(t[60]);
            f.traceA = Integer.parseInt(t[61]);
            f.traceB = Integer.parseInt(t[62]);
            f.skipped = Long.parseLong(t[63]);
        }
        if (f.state < 0 || f.state > (f.extended ? 8 : 5))
            throw new IllegalArgumentException("未知车端状态");
        for (int i = 0; i < 3; i++) {
            int j = 6 + 5 * i;
            Ray r = new Ray();
            r.stamp = Long.parseLong(t[j]);
            r.mm = Integer.parseInt(t[j + 1]);
            r.x = Integer.parseInt(t[j + 2]);
            r.y = Integer.parseInt(t[j + 3]);
            r.angle = Integer.parseInt(t[j + 4]);
            f.rays[i] = r;
        }
        return f;
    }
}
