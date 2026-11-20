package org.embeddedcars.patrol;

import android.content.Context;
import android.graphics.*;
import android.view.*;
import java.util.List;

final class MapView extends View {
    interface Tap {
        void add(int id);
    }
    final Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
    Grid grid;
    Pilot pilot;
    Tap tap;
    float zoom = 1, panX, panY, lastX, lastY, downX, downY;
    boolean moved;
    final ScaleGestureDetector scale;
    MapView(Context c) {
        super(c);
        scale = new ScaleGestureDetector(c, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            public boolean onScale(ScaleGestureDetector d) {
                zoom = Math.max(.22f, Math.min(5f, zoom * d.getScaleFactor()));
                moved = true;
                invalidate();
                return true;
            }
        });
        setContentDescription("巡逻地图，点按添加途经点，拖动平移，双指缩放");
    }
    float unit() {
        return Math.min(getWidth(), getHeight()) / 18f * zoom;
    }
    float sx(double y) {
        return (float) (getWidth() / 2f + panX - y * unit() / Grid.CELL);
    }
    float sy(double x) {
        return (float) (getHeight() / 2f + panY - x * unit() / Grid.CELL);
    }
    protected void onDraw(Canvas c) {
        c.drawColor(0xffdbe4e4);
        if (grid == null || getWidth() == 0)
            return;
        float u = unit(), h = u / 2;
        int minA = Math.max(0, Grid.MID + (int) Math.floor((sy(0) - getHeight()) / u) - 1);
        int maxA = Math.min(Grid.N - 1, Grid.MID + (int) Math.ceil(sy(0) / u) + 1);
        int minB = Math.max(0, Grid.MID + (int) Math.floor((sx(0) - getWidth()) / u) - 1);
        int maxB = Math.min(Grid.N - 1, Grid.MID + (int) Math.ceil(sx(0) / u) + 1);
        for (int a = minA; a <= maxA; a++)
            for (int b = minB; b <= maxB; b++) {
                int id = a * Grid.N + b, value = grid.evidence[id];
                float x = sx(Grid.yOf(id)), y = sy(Grid.xOf(id));
                p.setColor(value >= 2 ? 0xff23424d : value < 0 ? 0xfffafcf8 : 0xffdbe4e4);
                c.drawRect(x - h, y - h, x + h, y + h, p);
                if (b % 5 == 0 || a % 5 == 0) {
                    p.setColor(0x33759291);
                    p.setStrokeWidth(1);
                    if (a % 5 == 0)
                        c.drawLine(x - h, y, x + h, y, p);
                    if (b % 5 == 0)
                        c.drawLine(x, y - h, x, y + h, p);
                }
            }
        float x = sx(grid.y), y = sy(grid.x);
        p.setColor(0xff138975);
        p.setStrokeWidth(3);
        float px = x, py = y;
        for (int id : pilot.path) {
            float nx = sx(Grid.yOf(id)), ny = sy(Grid.xOf(id));
            c.drawLine(px, py, nx, ny, p);
            px = nx;
            py = ny;
        }
        int n = 0;
        for (int id : pilot.goals) {
            float gx = sx(Grid.yOf(id)), gy = sy(Grid.xOf(id));
            p.setColor(0xffad531f);
            c.drawCircle(gx, gy, 11, p);
            p.setTextSize(16);
            p.setColor(-1);
            p.setTextAlign(Paint.Align.CENTER);
            c.drawText(Integer.toString(++n), gx, gy + 5, p);
        }
        c.save();
        c.rotate((float) -grid.heading, x, y);
        Path arrow = new Path();
        arrow.moveTo(x, y - 16);
        arrow.lineTo(x + 11, y + 11);
        arrow.lineTo(x, y + 7);
        arrow.lineTo(x - 11, y + 11);
        arrow.close();
        p.setColor(0xff035d61);
        c.drawPath(arrow, p);
        c.restore();
        p.setTextAlign(Paint.Align.LEFT);
        p.setColor(0xff294849);
        p.setTextSize(22);
        c.drawLine(18, getHeight() - 18, 18 + 5 * u, getHeight() - 18, p);
        c.drawText("1 m", 18, getHeight() - 26, p);
    }
    public boolean onTouchEvent(MotionEvent e) {
        scale.onTouchEvent(e);
        float x = e.getX(), y = e.getY();
        switch (e.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                downX = lastX = x;
                downY = lastY = y;
                moved = false;
                return true;
            case MotionEvent.ACTION_MOVE:
                if (Math.hypot(x - downX, y - downY) > 10)
                    moved = true;
                if (moved && !scale.isInProgress()) {
                    panX += x - lastX;
                    panY += y - lastY;
                    invalidate();
                }
                lastX = x;
                lastY = y;
                return true;
            case MotionEvent.ACTION_UP:
                if (!moved && tap != null) {
                    int id = Grid.cell((sy(0) - y) / unit() * Grid.CELL, (sx(0) - x) / unit() * Grid.CELL);
                    if (id >= 0)
                        tap.add(id);
                    performClick();
                }
                return true;
            default:
                return true;
        }
    }
    public boolean performClick() {
        super.performClick();
        return true;
    }
}
