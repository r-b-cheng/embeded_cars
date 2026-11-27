package org.embeddedcars.patrol;

import android.content.*;
import android.database.Cursor;
import android.net.Uri;
import android.os.*;
import android.util.Base64;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.*;
import org.json.JSONObject;

/** Opt-in ADB shell bridge. All motion state belongs to the activity UI thread. */
public final class PilotProvider extends ContentProvider {
    static volatile SmartPilot active;
    public boolean onCreate() { return true; }
    public Bundle call(String method, String arg, Bundle extras) {
        int uid = Binder.getCallingUid();
        if (uid != 2000 && uid != 0) throw new SecurityException("ADB shell only");
        FutureTask<String> work = new FutureTask<>(() -> {
            SmartPilot p = active;
            if (p == null || p.mode != SmartPilot.Mode.ADB) throw new IllegalStateException("手机未开启 ADB 智能模式");
            if ("snapshot".equals(method)) return p.observe().toString();
            if ("stop".equals(method)) { p.app.halt("电脑请求停车"); return "{\"stopped\":true}"; }
            if (!"decide".equals(method) || arg == null || arg.length() > 4096)
                throw new IllegalArgumentException("请求无效");
            String json = new String(Base64.decode(arg,Base64.URL_SAFE),StandardCharsets.UTF_8);
            return Telemetry.object("accepted",p.decide(new JSONObject(json))).toString();
        });
        new Handler(Looper.getMainLooper()).post(work);
        Bundle b = new Bundle();
        try {
            String json = work.get(2,TimeUnit.SECONDS);
            b.putString("payload",Base64.encodeToString(json.getBytes(StandardCharsets.UTF_8),
                Base64.URL_SAFE | Base64.NO_WRAP));
        } catch (Exception e) {
            work.cancel(false);
            Throwable why = e.getCause() == null ? e : e.getCause();
            b.putString("error",why.getMessage());
        }
        return b;
    }
    public Cursor query(Uri u,String[] p,String s,String[] a,String o){throw new UnsupportedOperationException();}
    public String getType(Uri u){return null;}
    public Uri insert(Uri u,ContentValues v){throw new UnsupportedOperationException();}
    public int delete(Uri u,String s,String[] a){throw new UnsupportedOperationException();}
    public int update(Uri u,ContentValues v,String s,String[] a){throw new UnsupportedOperationException();}
}
