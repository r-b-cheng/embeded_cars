package org.embeddedcars.patrol;

import android.app.*;
import android.graphics.Color;
import android.content.Intent;
import android.text.InputType;
import android.net.*;
import android.os.*;
import android.view.*;
import android.widget.*;
import android.util.AtomicFile;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class MainActivity extends Activity {
    Grid grid = new Grid();
    Pilot pilot = new Pilot();
    MapView map;
    TextView info, ranges;
    Button link;
    final Handler ui = new Handler(Looper.getMainLooper());
    final ExecutorService worker = Executors.newSingleThreadExecutor();
    volatile Socket socket;
    volatile ServerSocket listener;
    volatile int session;
    int seq, taskId;
    String task;
    DriveLog log;
    static final int EXPORT_LOG = 7;
    boolean online, connecting;
    char manual = 'S', desired = 'S';
    String address;
    Frame last;
    AtomicFile file;
    long saved, noRangeSince;
    int dp(int n) {
        return Math.round(getResources().getDisplayMetrics().density * n);
    }

    public void onCreate(Bundle b) {
        super.onCreate(b);
        file = new AtomicFile(new File(getFilesDir(), "survey-v1"));
        address = getPreferences(0).getString("host", "192.168.4.1");
        log = new DriveLog(getFilesDir());
        load();
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(1);
        page.setBackgroundColor(0xfff8faf7);
        page.setOnApplyWindowInsetsListener((v, in) -> {
            page.setPadding(dp(10), in.getSystemWindowInsetTop(), dp(10), in.getSystemWindowInsetBottom());
            return in;
        });
        setContentView(page);
        LinearLayout head = row(page);
        TextView title = new TextView(this);
        title.setText("巡逻地图");
        title.setTextSize(23);
        title.setTextColor(0xff173e42);
        head.addView(title, new LinearLayout.LayoutParams(0, -2, 1));
        link = button(head, "连接", () -> {
            if (online || connecting)
                disconnect("已断开");
            else
                connectDialog();
        });
        info = new TextView(this);
        page.addView(info);
        map = new MapView(this);
        map.grid = grid;
        map.pilot = pilot;
        page.addView(map, new LinearLayout.LayoutParams(-1, 0, 1));
        map.tap = id -> {
            if (pilot.mode == Pilot.Mode.IDLE && pilot.goals.size() < 48) {
                pilot.goals.add(id);
                pilot.status = "已选 " + pilot.goals.size() + " 个途经点";
                render();
            }
        };
        ranges = new TextView(this);
        page.addView(ranges);
        LinearLayout routes = row(page);
        button(routes, "走路线", () -> start(Pilot.Mode.SURVEY));
        button(routes, "图上导航", () -> start(Pilot.Mode.ROUTE));
        button(routes, "自主探索", () -> start(Pilot.Mode.EXPLORE));
        LinearLayout drive = row(page);
        hold(drive, "左转", 'L');
        hold(drive, "前进", 'F');
        hold(drive, "右转", 'R');
        LinearLayout moves = row(page);
        button(moves, "持续前进", () -> {
            if (last == null) { halt("请先连接小车"); return; }
            halt("持续前进中 · 点停车结束");
            desired = manual = 'F';
        });
        button(moves, "定距前进", () -> targetDialog(false));
        button(moves, "定角转弯", () -> targetDialog(true));
        button(moves, "导出日志", this::exportLog);
        LinearLayout actions = row(page);
        button(actions, "清路线", () -> {
            halt("路线已清空");
            pilot.goals.clear();
            render();
        });
        button(actions, "回到车位", () -> {
            map.panX += (map.getWidth() / 2f - map.sx(grid.y));
            map.panY += (map.getHeight() / 2f - map.sy(grid.x));
            map.invalidate();
        });
        Button stop = button(actions, "停车", () -> halt("已请求停车"));
        stop.setTextColor(Color.rgb(156, 47, 38));
        render();
    }
    LinearLayout row(LinearLayout p) {
        LinearLayout r = new LinearLayout(this);
        r.setGravity(Gravity.CENTER_VERTICAL);
        p.addView(r, new LinearLayout.LayoutParams(-1, -2));
        return r;
    }
    Button button(LinearLayout row, String name, Runnable action) {
        Button b = new Button(this);
        b.setText(name);
        b.setTextSize(14);
        b.setPadding(0, 0, 0, 0);
        row.addView(b, new LinearLayout.LayoutParams(0, dp(44), 1));
        b.setOnClickListener(v -> action.run());
        return b;
    }
    void hold(LinearLayout row, String name, char motion) {
        Button b = button(row, name, () -> {});
        b.setOnTouchListener((v, e) -> {
            if (e.getActionMasked() == MotionEvent.ACTION_DOWN && online && last != null) {
                halt("手动 · 松手停车");
                manual = desired = motion;
                render();
                return true;
            }
            if (e.getActionMasked() == MotionEvent.ACTION_UP
                || e.getActionMasked() == MotionEvent.ACTION_CANCEL) {
                manual = desired = 'S';
                return true;
            }
            return true;
        });
    }
    void targetDialog(boolean turn) {
        halt("停车设置目标");
        if (last == null || !last.extended || !last.imuReady) {
            halt("定量运动需要新版固件和就绪的 IMU");
            return;
        }
        EditText input = new EditText(this);
        input.setSingleLine();
        input.setInputType(InputType.TYPE_CLASS_NUMBER | (turn ? InputType.TYPE_NUMBER_FLAG_SIGNED : 0));
        input.setText(turn ? "90" : "50");
        input.selectAll();
        new AlertDialog.Builder(this).setTitle(turn ? "相对转角（度）" : "前进距离（厘米）")
            .setMessage(turn ? "左转为正，右转为负，最多一圈。" : "输入 1 至 1000 厘米。")
            .setView(input).setNegativeButton("取消", null).setPositiveButton("开始", (d, w) -> {
                try {
                    int value = Integer.parseInt(input.getText().toString().trim());
                    if (value == 0 || value < (turn ? -360 : 1) || value > (turn ? 360 : 1000))
                        throw new NumberFormatException();
                    if (!online || last == null) { halt("连接已中断"); return; }
                    taskId = taskId % 65535 + 1;
                    task = (turn ? "A " : "D ") + taskId + " " + (turn ? value : value * 10);
                    pilot.status = turn ? "转动 " + value + "°" : "前进 " + value + " cm";
                    render();
                } catch (NumberFormatException e) { halt("数值超出范围"); }
            }).show();
    }
    void exportLog() {
        halt("正在导出日志");
        Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT).setType("text/tab-separated-values")
            .addCategory(Intent.CATEGORY_OPENABLE).putExtra(Intent.EXTRA_TITLE, "patrol-drive.tsv");
        startActivityForResult(intent, EXPORT_LOG);
    }
    @Override protected void onActivityResult(int request, int result, Intent data) {
        super.onActivityResult(request, result, data);
        if (request == EXPORT_LOG && result == RESULT_OK && data != null && data.getData() != null) {
            worker.execute(() -> {
                try (OutputStream out = getContentResolver().openOutputStream(data.getData(), "wt")) {
                    if (out == null) throw new IOException();
                    log.export(out);
                    ui.post(() -> halt("日志已导出"));
                } catch (IOException e) { ui.post(() -> halt("日志导出失败")); }
            });
        }
    }
    void halt(String why) {
        noRangeSince = 0;
        task = null;
        manual = desired = 'S';
        pilot.stop(why);
        render();
    }
    void start(Pilot.Mode m) {
        if (!online || last == null) {
            halt("请先连接小车");
            return;
        }
        noRangeSince = 0;
        task = null;
        manual = desired = 'S';
        pilot.start(m, grid, SystemClock.elapsedRealtime());
        render();
    }
    void connectDialog() {
        EditText field = new EditText(this);
        field.setSingleLine();
        field.setText(address);
        new AlertDialog.Builder(this)
            .setTitle("连接巡逻车")
            .setMessage("先打开手机的 2.4 GHz 热点。小车上线后会主动连接。")
            .setPositiveButton("等待小车", (d, w) -> mapChoice(null))
            .setNeutralButton("输入 IP", (d, w) -> new AlertDialog.Builder(this)
                .setTitle("小车或模拟器 IP").setView(field)
                .setPositiveButton("下一步", (a, b) -> mapChoice(field.getText().toString().trim()))
                .setNegativeButton("取消", null).show())
            .setNegativeButton("取消", null).show();
    }
    void mapChoice(String host) {
        new AlertDialog.Builder(this).setTitle("巡逻地图")
            .setMessage("复用原图时，将车放回原起点并朝原方向。")
            .setPositiveButton("新测区", (d, w) -> connect(host, false))
            .setNeutralButton("复用原图", (d, w) -> connect(host, true))
            .setNegativeButton("取消", null).show();
    }
    Socket newSocket() {
        ConnectivityManager cm = (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        for (Network n : cm.getAllNetworks()) {
            NetworkCapabilities c = cm.getNetworkCapabilities(n);
            if (c != null && c.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                try {
                    return n.getSocketFactory().createSocket();
                } catch (IOException ignored) {
                }
            }
        }
        return new Socket();
    }
    void connect(String host, boolean reuse) {
        if (host != null && host.isEmpty()) {
            halt("请输入小车或模拟器的地址");
            return;
        }
        disconnect(host == null ? "等待热点内的小车" : "正在连接");
        connecting = true;
        final ServerSocket server;
        try { server = host == null ? new ServerSocket() : null; }
        catch (IOException e) { disconnect("无法监听"); return; }
        listener = server;
        if (host != null) {
            address = host;
            getPreferences(0).edit().putString("host", address).apply();
        }
        final int generation = session;
        render();
        worker.execute(() -> {
            Socket candidate = null;
            try {
                if (server != null) {
                    server.setReuseAddress(true);
                    server.bind(new InetSocketAddress(7071));
                    candidate = server.accept();
                    server.close();
                } else {
                    candidate = new Socket();
                    try {
                        candidate.connect(new InetSocketAddress(host, 7070), 1500);
                    } catch (IOException first) {
                        close(candidate);
                        candidate = newSocket();
                        candidate.connect(new InetSocketAddress(host, 7070), 1800);
                    }
                }
                final Socket s = candidate;
                s.setSoTimeout(550);
                s.setTcpNoDelay(true);
                InputStream in = s.getInputStream();
                OutputStream out = s.getOutputStream();
                ui.post(() -> {
                    if (generation != session) {
                        close(s);
                        return;
                    }
                    socket = s;
                    online = true;
                    connecting = false;
                    // Wait for the first valid stopped response before replacing a saved map.
                    exchange(generation, in, out, true, reuse);
                });
            } catch (Exception e) {
                close(candidate);
                failed(generation, e);
            }
        });
    }
    void exchange(int generation, InputStream in, OutputStream out, boolean first, boolean reuse) {
        if (generation != session || !online)
            return;
        final boolean extended = first || last.extended;
        final String action = first ? "S" : task != null ? task : pilot.wireCommand(grid, desired);
        final int id = seq = (seq + 1) & 65535;
        worker.execute(() -> {
            try {
                String command = (extended ? "V " : "") + action;
                String line;
                try {
                    line = request(in, out, id, command);
                } catch (SocketTimeoutException e) {
                    if (!first) throw e;
                    command = "S"; // v1 firmware silently discards an unknown request
                    line = request(in, out, id, command);
                }
                Frame f = Frame.parse(line, id);
                log.append(id, command, line);
                ui.post(() -> {
                    if (generation != session)
                        return;
                    if (first) {
                        Grid next = new Grid();
                        if (reuse)
                            System.arraycopy(grid.evidence, 0, next.evidence, 0, next.evidence.length);
                        grid = next;
                        map.grid = grid;
                        if (!reuse)
                            pilot.goals.clear();
                        map.panX = map.panY = 0;
                        pilot.status = "已连接";
                    }
                    last = f;
                    grid.accept(f);
                    long now = SystemClock.elapsedRealtime();
                    desired = manual == 'S' ? pilot.step(grid, f, now) : manual;
                    if (f.state != 3) noRangeSince = 0;
                    else if (noRangeSince == 0) noRangeSince = now;
                    if (f.state == 8) halt("驱动线停车电平异常，请断开电机支路检查");
                    else if (f.state == 7 || f.state == 5 || f.state == 4)
                        halt(f.state == 7 ? "反馈异常停车 · 检查 IMU 和码盘" : "车端已停车，需重新操作");
                    else if (task != null && f.taskId == taskId && f.state == 6)
                        halt("目标已完成");
                    else if ((task != null || manual != 'S') && (f.state == 2 || (f.state == 3 && now - noRangeSince > 1800)))
                        halt(f.state == 2 ? "遇障停车" : "回波无效，已停车");
                    render();
                    if (now - saved > 5000) {
                        save();
                        saved = now;
                    }
                    ui.postDelayed(() -> exchange(generation, in, out, false, false), 100);
                });
            } catch (Exception e) {
                failed(generation, e);
            }
        });
    }
    static String request(InputStream in, OutputStream out, int id, String command) throws IOException {
        out.write((id + " " + command + "\n").getBytes(StandardCharsets.US_ASCII));
        out.flush();
        StringBuilder line = new StringBuilder();
        int c;
        while ((c = in.read()) != '\n') {
            if (c < 32 || c > 126 || line.length() >= 511) throw new IOException("遥测中断");
            line.append((char)c);
        }
        return line.toString();
    }
    void failed(int generation, Exception e) {
        ui.post(() -> {
            if (generation == session)
                disconnect("连接中断 · " + e.getClass().getSimpleName());
        });
    }
    static void close(Socket s) {
        if (s != null)
            try {
                s.close();
            } catch (IOException ignored) {
            }
    }
    void disconnect(String why) {
        ++session;
        online = connecting = false;
        if (listener != null) try { listener.close(); } catch (IOException ignored) {}
        listener = null;
        close(socket);
        socket = null;
        if (last != null)
            save();
        last = null;
        halt(why);
    }
    void render() {
        if (info == null)
            return;
        link.setText(online ? "断开" : connecting ? "取消" : "连接");
        info.setText((online ? "在线" : "离线") + "  ·  " + pilot.status + "  ·  " + grid.fix);
        String text = "灰色 未知　白色 可通行　深色 障碍";
        if (last != null) {
            String[] names = {"待命", "行驶", "障碍停车", "回波无效", "失联停车", "按键停车", "目标完成", "反馈异常", "驱动线异常"};
            text = names[last.state] + "   前 " + distance(0) + "   左 " + distance(1) + "   右 "
                + distance(2);
            if (last.extended)
                text += "\n左轮 " + last.speed + " mm/s　累计 " + last.travel + " mm　"
                    + (last.imuReady ? "相对航向 " + last.angle + "°" : "IMU 未就绪")
                    + (last.logFailed ? "　车端日志写入异常" : "");
        }
        ranges.setText(text);
        map.invalidate();
    }
    String distance(int i) {
        int mm = last.rays[i].mm;
        return mm < 0 ? "—" : (mm / 10) + " cm";
    }
    void save() {
        FileOutputStream fos = null;
        try {
            fos = file.startWrite();
            DataOutputStream out = new DataOutputStream(fos);
            out.writeInt(Grid.N);
            out.writeDouble(grid.x);
            out.writeDouble(grid.y);
            out.writeDouble(grid.heading);
            out.write(grid.evidence);
            out.flush();
            file.finishWrite(fos);
        } catch (IOException e) {
            if (fos != null)
                file.failWrite(fos);
        }
    }
    void load() {
        try (DataInputStream in = new DataInputStream(file.openRead())) {
            if (in.readInt() != Grid.N)
                return;
            Grid old = new Grid();
            old.x = in.readDouble();
            old.y = in.readDouble();
            old.heading = in.readDouble();
            in.readFully(old.evidence);
            grid = old;
        } catch (IOException ignored) {
        }
    }
    protected void onPause() {
        disconnect("已停止控制");
        super.onPause();
    }
    protected void onDestroy() {
        worker.shutdownNow();
        super.onDestroy();
    }
}
