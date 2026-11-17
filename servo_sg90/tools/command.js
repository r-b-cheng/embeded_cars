const { fs, path, local, image, checkBoard, mailbox } = require('./common.js');
const action = process.argv[3] || '1500';
const target = Number(action);
const amplitude = Number(process.argv[4] || 400);
if (!['stop', 'read', 'sweep'].includes(action) && (!Number.isInteger(target) || target < 1000 || target > 2200)) throw new Error('Use read, stop, sweep [amplitude], or 1000..2200');
if (action === 'sweep' && (!Number.isInteger(amplitude) || amplitude < 50 || amplitude > 400)) throw new Error('Sweep amplitude must be 50..400 us around 1500 us');
const serial = checkBoard();
const ds = initScripting({ logFile: path.join(local, 'command_debugserver.log') });
ds.setScriptingTimeout(20000);
let session;
let api;
try {
    ds.configure(path.join(__dirname, 'target.ccxml'));
    session = ds.openSession(/CORTEX_M4/i);
    session.target.connect();
    session.memory.verifyBinaryProgram(image, 0n);
    if (session.target.isHalted()) session.target.run(false);
    api = mailbox(session);
    const move = (pulse, delay = 850) => {
        const travelMs = Math.ceil(Math.abs(pulse - api.value('control_applied_us')) / 2) * 5;
        api.set('control_target_us', pulse);
        sleep(Math.max(delay, travelMs + 500));
        const state = api.snapshot();
        console.log('POSITION=' + JSON.stringify(state));
        if (!state.enabled || !state.active || !state.timer_enabled) throw new Error('Output stopped; sequence cancelled');
        if (state.applied_us !== pulse || state.match_pulse_us !== pulse || state.cfsr || state.hfsr) throw new Error('Position command did not settle');
    };
    if (action === 'stop') {
        api.set('control_enable', 0);
        sleep(50);
    } else if (action !== 'read') {
        api.set('control_target_us', action === 'sweep' ? 1500 : target);
        api.set('control_enable', 1);
        if (action === 'sweep') {
            move(1500, 1200);
            for (let repeat = 0; repeat < 2; repeat++) {
                for (const pulse of [1500 - amplitude, 1500, 1500 + amplitude, 1500]) move(pulse);
            }
        } else {
            move(target, 3500);
        }
    }
    const samples = [];
    for (let i = 0; i < (action === 'stop' ? 2 : 5); i++) {
        const state = api.snapshot();
        samples.push(state);
        console.log('STABILITY=' + JSON.stringify(state));
        if (action === 'stop' && (state.active || state.timer_enabled)) throw new Error('Stop failed');
        if (!['stop', 'read'].includes(action) && (state.target_us !== (action === 'sweep' ? 1500 : target) || state.applied_us !== state.target_us || state.match_pulse_us !== state.target_us || !state.timer_enabled)) throw new Error('Target changed during hold');
        sleep(1000);
    }
    const record = { time: new Date().toISOString(), serial, action, amplitude_us: action === 'sweep' ? amplitude : undefined, samples };
    fs.writeFileSync(path.join(local, 'last_command.json'), JSON.stringify(record, null, 2) + '\n');
    fs.appendFileSync(path.join(local, 'history.jsonl'), JSON.stringify(record) + '\n');
    session.target.disconnect();
} catch (error) {
    if (api && session && session.target.isConnected()) {
        api.set('control_enable', 0);
        if (session.target.isHalted()) session.target.run(false);
        sleep(50);
    }
    throw error;
} finally {
    ds.shutdown();
}
