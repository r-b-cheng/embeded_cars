package org.embeddedcars.patrol;

import java.io.*;
import java.nio.file.*;
import java.util.concurrent.*;

/** Hold the storage writer, saturate the queue, then verify recovery/export. */
public final class DriveLogTest {
    static class Slow extends DriveLog {
        final CountDownLatch writing = new CountDownLatch(1), release = new CountDownLatch(1);
        Slow(File dir) { super(dir); }
        @Override void write(String block) throws IOException {
            writing.countDown();
            try { release.await(); } catch (InterruptedException e) { throw new IOException(e); }
            super.write(block);
        }
    }
    public static void main(String[] args) throws Exception {
        Path temp = Files.createTempDirectory("patrol-log-");
        Slow log = new Slow(temp.toFile());
        try {
            log.append(1,"W S","start");
            assert log.writing.await(2,TimeUnit.SECONDS);
            long start = System.nanoTime();
            for (int i=0;i<20000;i++) log.append(i,"W F","frame");
            long ms = (System.nanoTime()-start)/1000000;
            assert ms < 650 && log.dropped.get() == 20000-DriveLog.CAPACITY;
            log.release.countDown();
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            log.export(out);
            String data = out.toString("UTF-8");
            assert data.contains("#GAP\t"+log.dropped.get()+"\n");
            assert data.split("\n").length==DriveLog.CAPACITY+3;
            log.append(42,"#STATUS","恢复");
            out.reset(); log.export(out);
            assert out.toString("UTF-8").contains("#STATUS\t恢复");
            System.out.printf("Blocked storage: 20,000 submissions in %d ms, %d gaps reported, export recovered.%n",
                ms,log.dropped.get());
        } finally {
            log.release.countDown(); log.closing=true; log.writer.join(2000);
            Files.deleteIfExists(log.current.toPath()); Files.deleteIfExists(temp);
        }
    }
}
