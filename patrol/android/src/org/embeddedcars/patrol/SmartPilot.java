package org.embeddedcars.patrol;

import android.os.SystemClock;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.concurrent.*;
import org.json.*;

final class SmartPilot {
    enum Mode { OFF, API, ADB }
    final MainActivity app;
    final ExecutorService network = Executors.newSingleThreadExecutor();
    Mode mode = Mode.OFF;
    String goal = "", url = "", model = "", key = "", ticket = "", previous = "开始任务";
    final String prompt;
    boolean thinking;
    int generation, steps;
    long issued, next;
    int x, y, heading;

    SmartPilot(MainActivity app) {
        this.app = app;
        try (InputStream in = app.getAssets().open("system-prompt.txt")) { prompt = read(in, 16384); }
        catch (IOException e) { throw new IllegalStateException(e); }
    }
    void cancel() {
        generation++;
        mode = Mode.OFF;
        thinking = false;
        ticket = "";
        key = "";
        PilotProvider.active = null;
    }
    void start(Mode m, String task, String endpoint, String engine, String secret) {
        app.halt("准备智能控制");
        if (app.last == null || !app.last.full || !app.last.imuReady || task.trim().isEmpty()) {
            app.halt("请连接双轮固件，等待 IMU 就绪并填写任务"); return;
        }
        if (m == Mode.API) try {
            URL u = new URL(endpoint);
            boolean loopback = u.getProtocol().equals("http") && u.getHost().equals("127.0.0.1");
            if ((!loopback && !u.getProtocol().equals("https")) || u.getUserInfo() != null ||
                u.getQuery() != null || u.getRef() != null || engine.trim().isEmpty())
                throw new IOException();
        } catch (IOException e) { app.halt("检查模型名和完整 HTTPS 接口地址"); return; }
        goal = task.trim(); url = endpoint; model = engine; key = secret;
        steps = 0; previous = "开始任务"; next = 0; mode = m;
        if (m == Mode.ADB) PilotProvider.active = this;
        app.pilot.status = m == Mode.ADB ? "ADB 智能模式 · 等待决策" : "手机 API 智能模式";
        app.note("SMART", app.pilot.status + " · " + goal);
        app.render();
    }
    boolean available() {
        Frame f = app.last;
        return mode != Mode.OFF && f != null && f.full && f.imuReady && f.fault == 0 &&
            f.state == 0 && "S".equals(f.applied) && app.task == null && f.pwmL == 0 && f.pwmR == 0 &&
            SystemClock.elapsedRealtime() - app.received < 500;
    }
    JSONObject observe() throws JSONException {
        if (app.last == null || !app.last.full) throw new IllegalStateException("遥测不可用");
        boolean ready = available();
        if (ready) {
            ticket = UUID.randomUUID().toString();
            issued = SystemClock.elapsedRealtime();
            x = app.last.x; y = app.last.y; heading = app.last.angle;
        }
        return Telemetry.object("goal",goal,"previous",previous,"steps",steps,"can_decide",ready,
            "ticket",ready ? ticket : "", "telemetry",Telemetry.snapshot(app.last,app.grid,app.sent));
    }
    String decide(JSONObject decision) throws JSONException {
        if (steps >= 32) { app.halt("已完成本轮 32 步"); throw new IllegalStateException("任务结束"); }
        if (!available() || ticket.isEmpty() || !ticket.equals(decision.getString("ticket")) ||
            SystemClock.elapsedRealtime() - issued > 30000 ||
            Math.hypot(app.last.x-x, app.last.y-y) > 40 ||
            Math.abs(Grid.wrap(app.last.angle-heading)) > 8)
            throw new IllegalArgumentException("观测已过期或车辆忙");
        ticket = "";
        Object raw = decision.get("value");
        String action = decision.getString("action"), reason = decision.getString("reason");
        if (!(raw instanceof Integer) || reason.length() > 200 || decision.length() != 4)
            throw new IllegalArgumentException("动作格式无效");
        int value = (Integer)raw;
        boolean drive = action.equals("forward") && value >= 10 && value <= 300;
        boolean turn = action.equals("turn") && value != 0 && value >= -90 && value <= 90;
        if (!drive && !turn && !(value == 0 && (action.equals("wait") || action.equals("stop"))))
            throw new IllegalArgumentException("动作或幅度越界");
        if (drive || turn) {
            for (int i=0; i<3; i++) {
                Frame.Ray r = app.last.rays[i];
                long age = (app.last.boardMs - r.stamp) & 0xffffffffL;
                int min = i == 0 ? (drive ? 400 : 180) : (turn && (value>0 ? i==1 : i==2) ? 300 : 160);
                if (age > (i == 0 ? 450 : 850) || r.mm < min || r.mm > 4000)
                    throw new IllegalArgumentException("回波过期或净空不足");
            }
        }
        previous = action + " " + value + " · " + reason;
        app.note("DECISION",previous);
        steps++;
        if (action.equals("stop")) app.halt("智能任务结束 · " + reason);
        else if (action.equals("wait")) {
            next = SystemClock.elapsedRealtime()+1200;
            app.pilot.status = "等待新观测 · " + reason;
        } else {
            app.taskId = app.taskId % 65535 + 1;
            app.task = (drive ? "D " : "A ") + app.taskId + " " + value;
            app.pilot.status = "智能控制 · " + previous;
        }
        app.render();
        return previous;
    }
    void completed() {
        app.task = null;
        previous += "（车端已完成）";
        app.note("COMPLETE",previous);
        app.pilot.status = previous;
        next = SystemClock.elapsedRealtime()+350;
    }
    void tick() {
        if (!available() || SystemClock.elapsedRealtime() < next) return;
        if (steps >= 32) { app.halt("已完成本轮 32 步"); return; }
        if (mode != Mode.API || thinking) return;
        final int g = generation;
        final String endpoint = url, engine = model, secret = key;
        try {
            final String input = observe().toString();
            thinking = true;
            app.pilot.status = "停车中 · 等待模型";
            network.execute(() -> {
                try {
                    JSONObject reply = ask(endpoint, engine, secret, input);
                    app.ui.post(() -> {
                        if (generation != g) return;
                        thinking = false;
                        try { decide(reply); }
                        catch (Exception e) { app.halt("模型动作未执行 · " + e.getMessage()); }
                    });
                } catch (Exception e) {
                    app.ui.post(() -> { if (generation == g) app.halt("模型请求失败 · " +
                        (e instanceof ApiError ? e.getMessage() : e.getClass().getSimpleName())); });
                }
            });
        } catch (JSONException e) { app.halt("无法生成观测"); }
    }
    static final class ApiError extends IOException {
        ApiError(int code) { super("HTTP " + code); }
    }
    JSONObject ask(String endpoint, String engine, String secret, String input) throws Exception {
        HttpURLConnection conn = (HttpURLConnection)new URL(endpoint).openConnection();
        try {
            conn.setInstanceFollowRedirects(false);
            conn.setConnectTimeout(5000); conn.setReadTimeout(20000);
            conn.setRequestMethod("POST"); conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type","application/json");
            if (!secret.isEmpty()) conn.setRequestProperty("Authorization","Bearer "+secret);
            JSONObject body = Telemetry.object("model",engine,"messages",Telemetry.array(
                Telemetry.object("role","system","content",prompt), Telemetry.object("role","user","content",input)));
            try (OutputStream out = conn.getOutputStream()) {
                out.write(body.toString().getBytes(StandardCharsets.UTF_8));
            }
            int code = conn.getResponseCode();
            if (code < 200 || code >= 300) throw new ApiError(code);
            try (InputStream in = conn.getInputStream()) {
                String content = new JSONObject(read(in,65536)).getJSONArray("choices")
                    .getJSONObject(0).getJSONObject("message").getString("content");
                JSONTokener tokener = new JSONTokener(content);
                Object result = tokener.nextValue();
                if (!(result instanceof JSONObject) || tokener.nextClean() != 0) throw new JSONException("JSON");
                return (JSONObject)result;
            }
        } finally { conn.disconnect(); }
    }
    static String read(InputStream input, int limit) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] block = new byte[2048];
        int n;
        while ((n=input.read(block)) != -1) {
            if (out.size()+n > limit) throw new IOException("响应过长");
            out.write(block,0,n);
        }
        return out.toString("UTF-8");
    }
}
