package org.embeddedcars.patrol;

import android.app.*;
import android.graphics.Color;
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
    int session, seq;
    boolean online, connecting;
    char manual = 'S', desired = 'S';
    String address;
    Frame last;
    AtomicFile file;
    long saved;
    int dp(int n) {
        return Math.round(getResources().getDisplayMetrics().density * n);
    }

    public void onCreate(Bundle b) {
        super.onCreate(b);
        file = new AtomicFile(new File(getFilesDir(), "survey-v1"));
        address = getPreferences(0).getString("host", "192.168.4.1");
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
    void halt(String why) {
        manual = desired = 'S';
        pilot.stop(why);
        render();
    }
    void start(Pilot.Mode m) {
        if (!online || last == null) {
            halt("请先连接小车");
            return;
        }
        manual = desired = 'S';
        pilot.start(m, grid, SystemClock.elapsedRealtime());
        render();
    }
    void connectDialog() {
        EditText field = new EditText(this);
        field.setSingleLine();
        field.setText(address);
        new AlertDialog.Builder(this)
            .setTitle("车载热点或模拟器地址")
            .setView(field)
            .setMessage("新测区从当前车位建图。复用上次地图时，把车放回原起点并朝原方向。")
            .setPositiveButton("新测区", (d, w) -> connect(field.getText().toString().trim(), false))
            .setNeutralButton("复用原图", (d, w) -> connect(field.getText().toString().trim(), true))
            .setNegativeButton("取消", null)
            .show();
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
        if (host.isEmpty()) {
            halt("请输入小车或模拟器的地址");
            return;
        }
        disconnect("正在连接");
        connecting = true;
        address = host;
        getPreferences(0).edit().putString("host", address).apply();
        final int generation = session;
        render();
        worker.execute(() -> {
            Socket candidate = null;
            try {
                final Socket s = candidate = newSocket();
                s.connect(new InetSocketAddress(host, 7070), 1800);
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
        final String action = first ? "S" : pilot.wireCommand(grid, desired);
        final int id = seq = (seq + 1) & 65535;
        worker.execute(() -> {
            try {
                out.write((id + " " + action + "\n").getBytes(StandardCharsets.US_ASCII));
                out.flush();
                StringBuilder line = new StringBuilder();
                int c;
                while ((c = in.read()) != '\n') {
                    if (c < 32 || c > 126 || line.length() >= 256)
                        throw new IOException("遥测中断");
                    line.append((char) c);
                }
                Frame f = Frame.parse(line.toString(), id);
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
                    if (f.state == 5 || f.state == 4)
                        halt("车端已停车，需重新操作");
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
            String[] names = {"待命", "行驶", "障碍停车", "回波无效", "失联停车", "按键停车"};
            text = names[last.state] + "   前 " + distance(0) + "   左 " + distance(1) + "   右 "
                + distance(2);
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
