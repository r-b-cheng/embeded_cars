package org.embeddedcars.patrol;

import java.io.*;
import java.util.*;

public final class NavigationTest {
    static Frame frame(int x, int y, int heading, long t, int front, int left, int right) {
        Frame f = new Frame();
        f.x = x;
        f.y = y;
        f.angle = heading;
        int[] mm = {front, left, right}, angle = {0, 35, -35};
        for (int i = 0; i < 3; i++) {
            Frame.Ray r = new Frame.Ray();
            r.mm = mm[i];
            r.stamp = t;
            r.x = x;
            r.y = y;
            r.angle = heading + angle[i];
            f.rays[i] = r;
        }
        return f;
    }
    static void mapping() {
        Grid g = new Grid();
        Frame f = frame(0, 0, 0, 1, 1000, -1, -1);
        g.accept(f);
        assert g.evidence[Grid.cell(1000, 0)] == 3;
        assert g.evidence[Grid.cell(600, 0)] < 0;
        g.accept(f);
        assert g.evidence[Grid.cell(1000, 0)] == 3;
        g.accept(frame(0, 0, 0, 2, -1, -1, -1));
        assert g.evidence[Grid.cell(0, 800)] == 0;
        // Synthetic stable vertical wall: correct normal error only.
        for (int y = -5; y <= 5; y++) g.evidence[Grid.cell(1000, y * 200)] = 8;
        g.accept(frame(200, 200, 0, 3, 950, -1, -1));
        assert g.x < 180 && g.x > 99 && Math.abs(g.y - 200) < 1;
        assert g.fix.equals("超声墙面校正");
        // Two distinct, perpendicular landmarks in the same scan.
        Grid corner = new Grid();
        corner.accept(frame(0, 0, 0, 0, -1, -1, -1));
        for (int i = -14; i <= 14; i++) {
            corner.evidence[Grid.cell(3000, i * 200)] = 8;
            corner.evidence[Grid.cell(i * 200, 1000)] = 8;
        }
        corner.accept(frame(300, 200, 0, 1, 2850, 1550, -1));
        assert corner.fix.equals("超声双向校正") && corner.x < 300 && corner.y < 200;
    }
    static void paths() {
        Grid g = new Grid();
        int origin = Grid.cell(0, 0), goal = Grid.cell(2000, 0);
        assert g.plan(origin, goal, false, false).isEmpty();
        for (int x = -400; x <= 2400; x += 200)
            for (int y = -1000; y <= 1000; y += 200) g.evidence[Grid.cell(x, y)] = -5;
        g.evidence[Grid.cell(1000, 0)] = 8;
        List<Integer> route = g.plan(origin, goal, false, false);
        assert !route.isEmpty() && route.get(route.size() - 1) == goal;
        assert route.stream().anyMatch(p -> Math.abs(Grid.yOf(p)) >= 400);
        for (int id : route) assert g.clear(id);
        int frontier = g.plan(origin, -1, true, false).get(0);
        assert g.clear(frontier);
        Pilot p = new Pilot();
        Frame f = frame(0, 0, 0, 1, 1500, 1000, 1000);
        p.goals.add(goal);
        p.start(Pilot.Mode.ROUTE, g, 1);
        assert p.step(g, f, 10) == 'F';
        f.rays[0].mm = 300;
        assert p.step(g, f, 200) == 'S';
        assert p.step(g, f, 300) != 'F'; // previous approach excluded
        f.rays[1].mm = -1;
        p.step(g, f, 500);
        p.step(g, f, 4100);
        assert p.mode == Pilot.Mode.IDLE;
    }

    /* Same app pilot and parser, driven by accelerated native simulation. */
    static void serve(String mission) throws Exception {
        BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
        Grid grid = new Grid();
        Pilot pilot = new Pilot();
        String s;
        int tick = 0;
        boolean started = false;
        while ((s = in.readLine()) != null) {
            Frame f = Frame.parse(s, 42);
            grid.accept(f);
            if (!started && tick >= 7) {
                if (!mission.equals("explore"))
                    pilot.goals.add(Grid.cell(mission.equals("route") ? 800 : 1600, 0));
                pilot.start(
                    mission.equals("explore") ? Pilot.Mode.EXPLORE : Pilot.Mode.SURVEY, grid, tick * 100L);
                started = true;
            }
            char cmd = pilot.step(grid, f, tick * 100L);
            System.out.println(
                pilot.wireCommand(grid, cmd) + "|" + Math.round(grid.x) + "|" + pilot.status);
            System.out.flush();
            tick++;
        }
    }
    public static void main(String[] args) throws Exception {
        if (args.length > 0) {
            serve(args[0]);
            return;
        }
        mapping();
        paths();
        System.out.println("Android mapping, landmark correction and replanning passed");
    }
}
