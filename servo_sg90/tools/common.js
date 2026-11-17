// Run through the TI CCS Scripting launcher, which provides initScripting/sleep.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const root = path.resolve(__dirname, '..');
const local = path.join(root, 'local');
const image = path.join(root, 'build/servo_sg90_hold.bin');
const metadata = JSON.parse(fs.readFileSync(path.join(root, 'build/servo_sg90_hold.symbols.json'), 'utf8'));
const sha256 = file => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
if (sha256(image) !== metadata.image_sha256) throw new Error('Binary and symbol metadata differ; rebuild with make');
fs.mkdirSync(local, { recursive: true });

function checkBoard() {
    const expected = process.env.TM4C_SERIAL;
    if (!expected) throw new Error('Set TM4C_SERIAL to the intended healthy board serial before connecting');
    if (process.platform !== 'linux') throw new Error('The serial preflight is verified on Linux; on Windows use the documented LM Flash Programmer procedure');
    const devices = [];
    for (const entry of fs.readdirSync('/sys/bus/usb/devices')) {
        const dir = path.join('/sys/bus/usb/devices', entry);
        try {
            const read = name => fs.readFileSync(path.join(dir, name), 'utf8').trim();
            if (read('idVendor') === '1cbe' && read('idProduct') === '00fd') devices.push(read('serial'));
        } catch (error) {
            if (error.code !== 'ENOENT' && error.code !== 'ENOTDIR') throw error;
        }
    }
    if (devices.length !== 1 || devices[0] !== expected) throw new Error('Expected only board ' + expected + '; detected ' + JSON.stringify(devices));
    return expected;
}

function mailbox(session) {
    const symbols = metadata.symbols;
    for (const key of ['control_target_us', 'control_enable', 'control_active', 'control_applied_us', 'control_millis']) {
        if (!Number.isInteger(symbols[key]) || symbols[key] < 0x20000000 || symbols[key] >= 0x20008000) throw new Error('Invalid mailbox symbol: ' + key);
    }
    const read = address => session.memory.readOne(BigInt(address), 32);
    const value = key => Number(read(symbols[key]));
    const set = (key, val) => session.memory.write(BigInt(symbols[key]), val, 32);
    const snapshot = () => ({
        millis: value('control_millis'), enabled: value('control_enable'), active: value('control_active'),
        target_us: value('control_target_us'), applied_us: value('control_applied_us'),
        match_pulse_us: (319999 - Number(read(0x40037034))) / 16,
        timer_enabled: (read(0x4003700c) & 0x100n) !== 0n,
        cfsr: Number(read(0xe000ed28)), hfsr: Number(read(0xe000ed2c)),
    });
    return { set, value, snapshot };
}

module.exports = { fs, path, root, local, image, sha256, checkBoard, mailbox };
