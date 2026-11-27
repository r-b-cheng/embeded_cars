package org.embeddedcars.patrol;

import org.json.*;

final class Telemetry {
    static JSONObject object(Object... pairs) throws JSONException {
        JSONObject result = new JSONObject();
        for (int i = 0; i < pairs.length; i += 2) result.put((String)pairs[i], pairs[i+1]);
        return result;
    }
    static JSONArray array(Object... values) {
        JSONArray a = new JSONArray();
        for (Object v : values) a.put(v);
        return a;
    }
    static JSONObject snapshot(Frame f, Grid g, String command) throws JSONException {
        JSONArray rays = new JSONArray(), accel = new JSONArray(), gyro = new JSONArray();
        for (int i = 0; i < 3; i++) {
            Frame.Ray r = f.rays[i];
            rays.put(object("sector", new String[]{"front","left","right"}[i], "stamp_ms",r.stamp,
                "range_mm",r.mm,"bearing_deg",f.bearing[i],"world_deg",r.angle,"origin_mm",array(r.x,r.y)));
            accel.put(f.imu[i] / 16384.0);
            gyro.put(f.imu[3+i] / 131.0);
        }
        JSONArray rows = new JSONArray();
        int cx = (int)Math.round(g.x / Grid.CELL), cy = (int)Math.round(g.y / Grid.CELL);
        for (int x = cx-5; x <= cx+5; x++) {
            StringBuilder row = new StringBuilder();
            for (int y = cy-5; y <= cy+5; y++) {
                int id = Grid.cell(x*Grid.CELL, y*Grid.CELL);
                int e = id < 0 ? 0 : g.evidence[id];
                row.append(x == cx && y == cy ? '@' : e >= 3 ? '#' : e < 0 ? '.' : '?');
            }
            rows.put(row.toString());
        }
        return object("board_ms",f.boardMs,"state",Frame.STATES[f.state],"error",Frame.FAULTS[f.fault],
            "request",command,"applied",f.applied,"task_id",f.taskId,"task_target",f.target,
            "pwm_pct",array(f.pwmL,f.pwmR),"sonar",rays,"servo_target_deg",f.servo,
            "imu_ready",f.imuReady,"imu_ms",f.imuMs,"accel_g",accel,"gyro_deg_s",gyro,
            "pose_mm_deg",array(f.x,f.y,f.angle),"wheel_rpm",array(f.leftRpm10/10.0,f.rightRpm10/10.0),
            "wheel_mm_s",array(f.speed,f.rightSpeed),"wheel_counts",array(f.pulses,f.rightPulses),
            "wheel_mm",array(f.travel,f.rightTravel),"center_mm_s",f.centerSpeed,"center_mm",f.centerTravel,
            "uart_errors",f.uartErrors,"radio_resets",f.resets,"invalid_requests",f.badRequests,
            "max_control_interval_ms",f.maxTick,"flash_write_failed",f.logFailed,
            "flash_event_id_ms_code",array(f.eventId,f.eventMs,f.eventCode),
            "trace_id_ms_kind_a_b",array(f.traceId,f.traceMs,f.traceKind,f.traceA,f.traceB),"trace_skipped",f.skipped,
            "map_pose_mm_deg",array(Math.round(g.x),Math.round(g.y),Math.round(g.heading)),
            "cell_mm",Grid.CELL,"map_origin_mm",array((cx-5)*Grid.CELL,(cy-5)*Grid.CELL),"map_rows",rows);
    }
}
