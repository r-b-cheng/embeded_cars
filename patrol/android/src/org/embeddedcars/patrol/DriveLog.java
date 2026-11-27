package org.embeddedcars.patrol;

import java.io.*;
import java.nio.charset.StandardCharsets;

/** Two bounded TSV segments, shared by live capture and offline export. */
final class DriveLog {
    final File current, previous;
    DriveLog(File dir) {
        current = new File(dir, "drive.tsv");
        previous = new File(dir, "drive.previous.tsv");
    }
    synchronized void append(int seq, String command, String frame) throws IOException {
        if (current.length() >= 2 * 1024 * 1024 && !current.renameTo(previous))
            throw new IOException("日志轮转失败");
        try (FileOutputStream out = new FileOutputStream(current, true)) {
            String row = System.currentTimeMillis() + "\t" + seq + "\t" + command + "\t" + frame + "\n";
            out.write(row.getBytes(StandardCharsets.US_ASCII));
        }
    }
    synchronized void export(OutputStream out) throws IOException {
        out.write("phone_unix_ms\tseq\tcommand\ttelemetry\n".getBytes(StandardCharsets.US_ASCII));
        byte[] buffer = new byte[8192];
        for (File f : new File[]{previous, current})
            if (f.exists()) try (InputStream in = new FileInputStream(f)) {
                int n;
                while ((n = in.read(buffer)) != -1) out.write(buffer, 0, n);
            }
    }
}
