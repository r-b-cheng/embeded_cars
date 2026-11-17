const { fs, path, local, image, sha256, checkBoard, mailbox } = require('./common.js');
const serial = checkBoard();
const backups = path.join(local, 'backups');
fs.mkdirSync(backups, { recursive: true });
const stamp = new Date().toISOString().replace(/[:.]/g, '-');
const backup = path.join(backups, `tm4c123_${serial}_${stamp}.bin`);
const ds = initScripting({ logFile: path.join(local, 'flash_debugserver.log') });
ds.setScriptingTimeout(30000);
let session;
try {
    ds.configure(path.join(__dirname, 'target.ccxml'));
    session = ds.openSession(/CORTEX_M4/i);
    session.target.connect();
    session.target.halt();
    const did1 = session.memory.readOne(0x400fe004n, 32);
    if (did1 !== 0x10a1606cn) throw new Error('Unexpected MCU; this image targets TM4C123GH6PM');
    session.memory.saveBinary(0n, 0x10000, backup);
    if (fs.statSync(backup).size !== 0x40000) throw new Error('Expected complete 256 KiB backup');
    const record = { serial, backup, backup_sha256: sha256(backup), image_sha256: sha256(image), status: 'backed_up' };
    fs.writeFileSync(path.join(local, 'last_flash.json'), JSON.stringify(record, null, 2) + '\n');
    checkBoard();
    session.memory.loadBinaryProgram(image, 0n);
    session.memory.verifyBinaryProgram(image, 0n);
    session.target.reset('System Reset');
    if (session.target.isHalted()) session.target.run(false);
    sleep(500);
    const idle = mailbox(session).snapshot();
    if (idle.enabled || idle.active || idle.timer_enabled || idle.cfsr || idle.hfsr) throw new Error('Unexpected boot state: ' + JSON.stringify(idle));
    record.status = 'verified_idle';
    record.idle = idle;
    fs.writeFileSync(path.join(local, 'last_flash.json'), JSON.stringify(record, null, 2) + '\n');
    console.log('FLASH_VERIFIED_IDLE=' + JSON.stringify(idle));
    session.target.disconnect();
} finally {
    ds.shutdown();
}
