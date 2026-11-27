package org.embeddedcars.patrol;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

/** Control submits without file I/O. Only the writer touches the live file. */
class DriveLog {
    static final int CAPACITY = 2048;
    final File current, previous, legacy;
    private final ArrayBlockingQueue<Object> queue = new ArrayBlockingQueue<>(CAPACITY);
    final AtomicLong dropped = new AtomicLong();
    private final AtomicLong unreported = new AtomicLong();
    volatile boolean diskError, closing;
    final Thread writer;

    DriveLog(File dir) {
        current = new File(dir,"drive-v3.tsv");
        legacy = new File(dir,"drive.tsv");
        previous = new File(dir,"drive.previous.tsv");
        writer = new Thread(this::drain,"patrol-log");
        writer.setDaemon(true);
        writer.start();
    }
    void append(int seq, String command, String frame) {
        String line = System.currentTimeMillis()+"\t"+seq+"\t"+command+"\t"+
            frame.replace('\t',' ').replace('\r',' ').replace('\n',' ')+"\n";
        if (!queue.offer(line)) loss(1);
    }
    private void loss(long n) { dropped.addAndGet(n); unreported.addAndGet(n); }
    void write(String block) throws IOException {
        try (OutputStream out = new FileOutputStream(current,true)) {
            out.write(block.getBytes(StandardCharsets.UTF_8));
        }
    }
    private void drain() {
        while (!closing || !queue.isEmpty()) {
            try {
                Object item = queue.poll(250,TimeUnit.MILLISECONDS);
                if (item == null) continue;
                StringBuilder batch = new StringBuilder();
                long missing = unreported.getAndSet(0);
                if (missing != 0) batch.append(System.currentTimeMillis()).append("\t0\t#GAP\t").append(missing).append('\n');
                int rows = 0;
                while (item instanceof String) {
                    batch.append((String)item);
                    rows++;
                    if (rows == 64) { item = null; break; }
                    item = queue.poll();
                }
                try {
                    if (batch.length() != 0) { write(batch.toString()); diskError = false; }
                } catch (IOException e) {
                    diskError = true;
                    unreported.addAndGet(missing);
                    loss(rows);
                }
                if (item instanceof CountDownLatch) ((CountDownLatch)item).countDown();
            } catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
        }
    }
    void export(OutputStream out) throws IOException {
        CountDownLatch done = new CountDownLatch(1);
        try {
            if (!queue.offer(done,5,TimeUnit.SECONDS) || !done.await(5,TimeUnit.SECONDS))
                throw new IOException("写入仍在进行");
        } catch (InterruptedException e) { Thread.currentThread().interrupt(); throw new IOException(e); }
        if (diskError) throw new IOException("本地存储异常");
        out.write("phone_unix_ms\tseq\tcommand\ttelemetry\n".getBytes(StandardCharsets.UTF_8));
        byte[] buf = new byte[8192];
        for (File f : new File[]{previous,legacy,current}) if (f.exists()) {
            long remaining = f.length();
            try (InputStream in = new FileInputStream(f)) {
                while (remaining > 0) {
                    int n = in.read(buf,0,(int)Math.min(buf.length,remaining));
                    if (n < 0) throw new EOFException();
                    out.write(buf,0,n); remaining -= n;
                }
            }
        }
    }
}
