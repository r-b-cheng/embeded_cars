package org.embeddedcars.patrol;

import java.util.ArrayList;

final class Pilot {
    enum Mode { IDLE, ROUTE, SURVEY, EXPLORE }
    Mode mode = Mode.IDLE;
    final ArrayList<Integer> goals = new ArrayList<>(), path = new ArrayList<>();
    String status = "待命";
    private long progressAt, noEchoSince, blockUntil;
    private double progressX, progressY, scanTurn, lastHeading;
    private boolean searching;

    void stop(String why) {
        mode = Mode.IDLE;
        path.clear();
        status = why;
    }
    void start(Mode m, Grid g, long now) {
        path.clear();
        if (m != Mode.EXPLORE && goals.isEmpty()) {
            stop("先点地图添加途经点");
            return;
        }
        mode = m;
        searching = false;
        blockUntil = noEchoSince = 0;
        g.temporaryBlock = -1;
        progressAt = now;
        progressX = g.x;
        progressY = g.y;
    }

    char step(Grid g, Frame f, long now) {
        if (mode == Mode.IDLE)
            return 'S';
        if (f.state == 5 || f.state == 4) {
            stop("车端已停止，请重新启动任务");
            return 'S';
        }
        boolean invalid = false;
        for (Frame.Ray r : f.rays)
            if (r.mm < 20 || r.mm > 4000)
                invalid = true;
        if (invalid) {
            if (noEchoSince == 0)
                noEchoSince = now;
            status = "等待有效回波";
            if (now - noEchoSince > 3500)
                stop("回波持续缺失，任务已暂停");
            return 'S';
        }
        noEchoSince = 0;
        if (Math.hypot(g.x - progressX, g.y - progressY) > 180) {
            progressAt = now;
            progressX = g.x;
            progressY = g.y;
        }
        if (now - progressAt > 24000) {
            stop("未能继续推进，任务已暂停");
            return 'S';
        }
        int here = Grid.cell(g.x, g.y);
        if (here < 0) {
            stop("已到地图边界");
            return 'S';
        }

        if (now >= blockUntil)
            g.temporaryBlock = -1;
        // Invalidate a route when new sonar evidence blocks any remaining cell.
        for (int id : path)
            if (!g.clear(id)) {
                path.clear();
                break;
            }
        while (!path.isEmpty() && near(g, path.get(0))) path.remove(0);
        if (mode != Mode.EXPLORE) {
            while (!goals.isEmpty() && near(g, goals.get(0))) {
                goals.remove(0);
                path.clear();
            }
            if (goals.isEmpty()) {
                stop("路线已完成");
                return 'S';
            }
        }
        if (path.isEmpty()) {
            int target = mode == Mode.EXPLORE ? -1 : goals.get(0);
            if (target == here && target >= 0)
                path.add(target);
            else
                path.addAll(g.plan(here, target, mode == Mode.EXPLORE, mode == Mode.SURVEY));
            if (path.isEmpty())
                return search(g, f);
        }
        searching = false;
        int next = path.get(0);
        double desired = Math.toDegrees(Math.atan2(Grid.yOf(next) - g.y, Grid.xOf(next) - g.x));
        double error = Grid.wrap(desired - g.heading);
        char cmd = Math.abs(error) > 17 ? (error > 0 ? 'L' : 'R') : 'F';
        if (cmd == 'F' && f.rays[0].mm < 400) {
            // Exclude just this approach while searching for a way around it.
            // Do not turn the same echo into new permanent map evidence.
            g.temporaryBlock = next;
            blockUntil = now + 4000;
            path.clear();
            status = "前方受阻，重新规划";
            return 'S';
        }

        status = mode == Mode.EXPLORE ? "自主探索" : "执行路线";
        return cmd;
    }

    String wireCommand(Grid g, char motion) {
        if (mode != Mode.IDLE && motion != 'S' && !searching && !path.isEmpty())
            return g.waypoint(path.get(0));
        return Character.toString(motion);
    }

    private char search(Grid g, Frame f) {
        if (!searching) {
            searching = true;
            scanTurn = 0;
            lastHeading = g.heading;
        }
        scanTurn += Math.abs(Grid.wrap(g.heading - lastHeading));
        lastHeading = g.heading;
        if (scanTurn > 350) {
            stop("扫描完成，未找到可达路径");
            return 'S';
        }
        status = "原地扫描可通行方向";
        return f.rays[1].mm >= 300 ? 'L' : 'R';
    }
    private boolean near(Grid g, int id) {
        return Math.hypot(Grid.xOf(id) - g.x, Grid.yOf(id) - g.y) < 85;
    }
}
