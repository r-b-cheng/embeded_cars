package org.embeddedcars.patrol;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/** World x = initial forward, y = initial left, all distances in mm. */
final class Grid {
    static final int N = 201, CELL = 200, MID = N / 2;
    final byte[] evidence = new byte[N * N];
    final boolean[] visited = new boolean[N * N];
    final long[] seen = {-1, -1, -1};
    double x, y, heading, dx, dy, rotation;
    String fix = "轮速推算";
    private double keyX, keyY, keyAngle;
    private boolean anchored;

    static int cell(double x, double y) {
        int a = MID + (int) Math.round(x / CELL), b = MID + (int) Math.round(y / CELL);
        return a < 0 || b < 0 || a >= N || b >= N ? -1 : a * N + b;
    }
    static int xOf(int id) {
        return (id / N - MID) * CELL;
    }
    static int yOf(int id) {
        return (id % N - MID) * CELL;
    }
    static double wrap(double d) {
        return Math.IEEEremainder(d, 360);
    }

    private double tx(double a, double b) {
        return dx + a * Math.cos(rotation) - b * Math.sin(rotation);
    }
    private double ty(double a, double b) {
        return dy + a * Math.sin(rotation) + b * Math.cos(rotation);
    }

    String waypoint(int cell) {
        double a = xOf(cell) - dx, b = yOf(cell) - dy;
        return "G " + Math.round(a * Math.cos(rotation) + b * Math.sin(rotation)) + " "
            + Math.round(-a * Math.sin(rotation) + b * Math.cos(rotation));
    }

    void accept(Frame f) {
        if (!anchored) {
            rotation = Math.toRadians(-f.angle);
            dx = -tx(f.x, f.y);
            dy = -ty(f.x, f.y);
            anchored = true;
        }
        heading = wrap(f.angle + Math.toDegrees(rotation));
        x = tx(f.x, f.y);
        y = ty(f.x, f.y);
        int here = cell(x, y);
        if (here >= 0) {
            visited[here] = true;
            evidence[here] = -8;
        }
        // Match a complete set to the previous map before inserting any of it.
        for (int i = 0; i < 3; i++)
            if (f.rays[i].stamp == seen[i])
                return;
        if (Math.hypot(x - keyX, y - keyY) >= 180 || Math.abs(wrap(heading - keyAngle)) >= 18) {
            correct(f);
            x = tx(f.x, f.y);
            y = ty(f.x, f.y);
            keyX = x;
            keyY = y;
            keyAngle = heading;
        }
        for (int i = 0; i < 3; ++i) {
            Frame.Ray r = f.rays[i];
            if (r.stamp == seen[i])
                continue;
            seen[i] = r.stamp;
            if (r.mm < 20 || r.mm > 4000)
                continue;
            double a = Math.toRadians(r.angle) + rotation;
            ray(tx(r.x, r.y), ty(r.x, r.y), a, r.mm);
        }
    }

    void ray(double ox, double oy, double a, int mm) {
        double c = Math.cos(a), s = Math.sin(a);
        int hit = cell(ox + c * mm, oy + s * mm), last = -1;
        for (int d = 0; d < mm; d += CELL / 3) {
            int id = cell(ox + c * d, oy + s * d);
            if (id >= 0 && id != hit && id != last)
                evidence[id] = (byte) Math.max(-8, evidence[id] - 1);
            last = id;
        }
        if (hit >= 0)
            evidence[hit] = (byte) Math.min(9, evidence[hit] + 3);
    }

    /* Match endpoints to established local wall segments. A straight wall
       constrains only its normal. Two independent normals constrain x and y. */
    private void correct(Frame f) {
        double xx = 0, xy = 0, yy = 0, bx = 0, by = 0;
        fix = "轮速推算";
        for (int i = 0; i < 3; ++i) {
            Frame.Ray r = f.rays[i];
            if (r.mm < 180 || r.mm > 3500)
                continue;
            double a = Math.toRadians(r.angle) + rotation;
            double ex = tx(r.x, r.y) + r.mm * Math.cos(a), ey = ty(r.x, r.y) + r.mm * Math.sin(a);
            int center = cell(ex, ey);
            if (center < 0)
                continue;
            double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
            int n = 0;
            for (int u = -2; u <= 2; u++)
                for (int v = -2; v <= 2; v++) {
                    int id = cell(xOf(center) + u * CELL, yOf(center) + v * CELL);
                    if (id < 0 || evidence[id] < 5)
                        continue;
                    double px = xOf(id) - ex, py = yOf(id) - ey;
                    sx += px;
                    sy += py;
                    sxx += px * px;
                    sxy += px * py;
                    syy += py * py;
                    n++;
                }
            if (n < 3)
                continue;
            sx /= n;
            sy /= n;
            double vx = sxx / n - sx * sx, vy = syy / n - sy * sy, cov = sxy / n - sx * sy;
            double spread = Math.hypot(vx - vy, 2 * cov);
            if (spread < CELL * CELL / 2.0 || spread < .80 * (vx + vy))
                continue;
            double tangent = .5 * Math.atan2(2 * cov, vx - vy);
            double nx = -Math.sin(tangent), ny = Math.cos(tangent);
            double residual = nx * sx + ny * sy;
            if (Math.abs(residual) > 240 || Math.abs(nx * Math.cos(a) + ny * Math.sin(a)) < .45)
                continue;
            xx += nx * nx;
            xy += nx * ny;
            yy += ny * ny;
            bx += nx * residual;
            by += ny * residual;
        }
        if (xx + yy < .5)
            return;
        boolean corner = xx * yy - xy * xy > .25;
        xx += .25;
        yy += .25;
        double det = xx * yy - xy * xy;
        double cx = (yy * bx - xy * by) / det, cy = (xx * by - xy * bx) / det;
        double scale = Math.min(.65, 100 / Math.max(1, Math.hypot(cx, cy)));
        dx += scale * cx;
        dy += scale * cy;
        fix = corner ? "超声双向校正" : "超声墙面校正";
    }

    /** Four-connected shortest path. Inflate obstacles by one cell around the chassis. */
    ArrayList<Integer> plan(int start, int goal, boolean frontier, boolean survey) {
        ArrayList<Integer> result = new ArrayList<>();
        if (start < 0 || (!frontier && goal < 0))
            return result;
        int[] parent = new int[N * N], queue = new int[N * N];
        Arrays.fill(parent, -1);
        int head = 0, tail = 1, found = -1;
        queue[0] = start;
        parent[start] = start;
        int[] offsets = {N, 1, -1, -N};
        while (head < tail) {
            int cur = queue[head++];
            if ((!frontier && cur == goal)
                || (frontier && cur != start && !visited[cur]
                    && Math.hypot(xOf(cur) - x, yOf(cur) - y) >= 600 && nearUnknown(cur))) {
                found = cur;
                break;
            }
            for (int off : offsets) {
                int next = cur + off;
                if (next < 0 || next >= parent.length || Math.abs(next % N - cur % N) > 1
                    || parent[next] >= 0 || (!survey && evidence[next] >= 0) || !clear(next))
                    continue;
                parent[next] = cur;
                queue[tail++] = next;
            }
        }
        for (int id = found; id >= 0 && id != start; id = parent[id]) result.add(id);
        Collections.reverse(result);
        return result;
    }

    int temporaryBlock = -1;
    boolean clear(int id) {
        if (id == temporaryBlock)
            return false;
        int row = id / N, col = id % N;
        if (row < 1 || col < 1 || row >= N - 1 || col >= N - 1)
            return false;
        for (int a = -1; a <= 1; a++)
            for (int b = -1; b <= 1; b++)
                if (evidence[id + a * N + b] >= 2)
                    return false;
        return true;
    }
    private boolean nearUnknown(int id) {
        return evidence[id + 1] == 0 || evidence[id - 1] == 0 || evidence[id + N] == 0
            || evidence[id - N] == 0;
    }
}
