package org.embeddedcars.patrol;

final class Frame {
    static final class Ray {
        long stamp;
        int mm, x, y, angle;
    }
    final Ray[] rays = new Ray[3];
    int state, x, y, angle;

    static Frame parse(String line, int seq) {
        String[] t = line.split(" ", -1);
        if (t.length != 21 || !t[0].equals("T") || Integer.parseInt(t[1]) != seq)
            throw new IllegalArgumentException("响应序号或格式错误");
        Frame f = new Frame();
        f.state = Integer.parseInt(t[2]);
        f.x = Integer.parseInt(t[3]);
        f.y = Integer.parseInt(t[4]);
        f.angle = Integer.parseInt(t[5]);
        if (f.state < 0 || f.state > 5)
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
