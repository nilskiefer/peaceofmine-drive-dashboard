"""Real camera aspect ratios and screen-sized views, without ROS or hardware."""
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
import unittest

DASHBOARD = Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'Chromium required')
class BrowserLayout(unittest.TestCase):
    def test_camera_geometry_and_screen_views(self):
        html = (DASHBOARD / 'index.html').read_text().split('  <script>')[0]
        html = re.sub(r'<link rel="stylesheet" href="assets/([^"]+)">', lambda match:
                      '<style>' + (DASHBOARD / match[1]).read_text() + '</style>', html)
        setup = '''
window.requestAnimationFrame = () => 0;
window.layoutTimers = [];
window.setInterval = (fn, ms) => {layoutTimers.push({fn, ms}); return 0;};
localStorage.setItem('peaceofmine.operator.preferences', JSON.stringify({
  cameras:{forward:'virtual',auxiliary:'raspberry:old-probe'}
}));
window.sent = [];
window.WebSocket = class {static OPEN=1; readyState=1; send(data) {sent.push(JSON.parse(data));}};
Object.defineProperty(document, 'hasFocus', {value: () => true});
Object.defineProperty(document, 'hidden', {value: false});
'''
        checks = '''
const assert = (value, label) => {if (!value) throw Error(label);};
const fits = element => {
  const b = element.getBoundingClientRect();
  return b.width > 0 && b.height > 0 && b.left >= -.5 && b.top >= -.5
    && b.right <= innerWidth + .5 && b.bottom <= innerHeight + .5;
};
const page = name => document.querySelector(`[data-workspace-page="${name}"]`).click();
state.connected = true;
state.drive.you_control_owner = state.drive.armed = true;
state.safety = {allowed: true, servo_allowed: true, mode: 'simulation'};
state.inspection = {phase: 'IDLE', active: false, points: []};
updateHud();
$('connection-overlay').classList.add('hidden');
assert(fits($('stop-controls')), 'STOP remains on screen');
const stage = $('camera-stage'), image = $('forward-network-camera');
const originalParent = cameraPanel.parentElement;
assert(preferences.cameras.auxiliary === 'off', 'legacy browser auxiliary preference is disabled');
assert(!$('probe-camera-panel').getClientRects().length && !$('camera-swap').getClientRects().length,
  'no probe camera panel or swap control');
assert(!$('aux-camera-source').getClientRects().length, 'camera setup offers no auxiliary view');
assert(!$('auxiliary-capture-device'), 'capture setup contains only the front camera');
await startCamera('auxiliary', 'raspberry:old-probe');
assert(cameraSources.auxiliary === 'off' && !$('aux-network-camera').hasAttribute('src'),
  'legacy auxiliary requests never open a stream');
preferences.cameras.forward = 'raspberry:test';
for (const [width, height] of [[640,480], [1920,1080], [480,640]]) {
  Object.defineProperty(image, 'naturalWidth', {value:width, configurable:true});
  Object.defineProperty(image, 'naturalHeight', {value:height, configurable:true});
  for (const angle of [0,90,180,270]) {
    preferences.cameraRotation.forward = angle;
    updateCameraVisibility();
    const b = cameraPanel.getBoundingClientRect(), f = image.getBoundingClientRect();
    const ratio = angle % 180 ? height / width : width / height;
    assert(fits(cameraPanel), 'front camera fits available screen');
    assert(Math.abs(cameraPanel.clientWidth / cameraPanel.clientHeight - ratio) < .025, 'frame matches rotated source');
    assert(Math.abs(f.width - (b.width-2)) < 1 && Math.abs(f.height - (b.height-2)) < 1,
      'full feed fills the frame without shrinking or cropping');
    const expectedWidth = Math.min(stage.clientWidth-2, (stage.clientHeight-2)*ratio);
    assert(Math.abs(b.width-2-expectedWidth) < 1, 'front camera uses the full available camera area');
  }
}
const layoutTick = layoutTimers.find(timer => timer.fn.name === 'tick').fn;
for (const phase of ['CONFIRMING','CONFIRMED','PROBING','COMPLETE','IDLE']) {
  state.inspection.phase = phase;
  layoutTick();
  assert(stage.dataset.focus === 'forward' && fits(cameraPanel) && $('camera-swap').hidden,
    'mission transitions keep the front camera visible');
}
assert(cameraPanel.parentElement === originalParent, 'mission transitions keep the camera focus target fixed');
const sharedCameras = {
  forward:{source:'virtual',rotation:0}, auxiliary:{source:'raspberry:old-probe',rotation:90},
  capture:{forward:{device:'auto',width:640,height:480,framerate:30,pixel_format:'mjpeg2rgb'},
    auxiliary:{device:'auto',width:640,height:480,framerate:30,pixel_format:'mjpeg2rgb'}}
};
operatorSettingsState({cameras:sharedCameras,saved:true});
assert(preferences.cameras.auxiliary === 'off' && !$('aux-network-camera').hasAttribute('src'),
  'shared legacy defaults cannot enable a probe camera');
saveCameraView('forward');
assert(sent.at(-1).value.auxiliary.source === 'raspberry:old-probe', 'front camera saves preserve unrelated shared legacy settings');
preferences.cameras.forward = 'virtual'; preferences.cameras.auxiliary = 'off';
preferences.cameraRotation.forward = 0;
stage.dataset.focus = 'forward'; stage.classList.add('single-camera');
updateCameraVisibility();
assert(Math.abs(cameraPanel.clientWidth / cameraPanel.clientHeight - 16/9) < .025, 'virtual camera restores aspect');
showNotice('Motor unavailable. Reconnect in Setup.');
applyCameraRotations();
assert(fits($('notice')) && fits(cameraPanel), 'warnings do not push the camera off screen');
const small = matchMedia('(max-width: 1000px), (max-height: 600px)').matches;
for (const view of ['mission','data','drive']) {
  page(view);
  const target = view === 'mission' ? document.querySelector('.mission')
    : view === 'data' ? document.querySelector('.dock') : cameraPanel;
  assert(fits(target), view + ' view fits screen');
  assert(fits($('notice')) && fits($('stop-controls')), 'warning and STOP stay accessible');
  assert(document.documentElement.scrollHeight <= innerHeight && document.documentElement.scrollWidth <= innerWidth,
    'no horizontal or vertical page overflow');
  if (small && view !== 'drive') assert(!cameraPanel.getClientRects().length, 'secondary view does not stack below camera');
}
// Exercise the normal drive gate with a synthetic wheel, never real hardware.
const wheel = {index:0, id:'Test wheel', axes:[0,-1,1], buttons:[]};
Object.defineProperty(navigator, 'getGamepads', {value:() => [wheel]});
selectedPadIndex=0; selectedPadId=wheel.id; preferences.input='wheel';
drivingPending=null;
state.drive.calibrating=false;
const drive = () => {lastDriveSent=-1000; inputLoop(); return sent.filter(m => m.type==='drive').at(-1);};
assert(drive()?.linear_x > 0, 'fixture permits driving with visible camera');
if (small) {
  page('mission');
  assert(drive()?.linear_x === 0 && !drive().deadman, 'hidden camera inhibits motion output');
  page('drive');
}
document.querySelector('[data-dock-tab="detector"]').click();
page('data');
$('detector-expand').click();
assert(fits(document.querySelector('.dock')) && fits($('stop-controls')), 'expanded chart keeps STOP on screen');
document.body.textContent = 'LAYOUT PASSED';
'''
        source = '\n'.join((DASHBOARD / name).read_text() for name in (
            'chart-4.4.8.umd.js', 'actuator-settings.js', 'app.js', 'marking.js', 'camera-config.js', 'layout.js'))
        html += '<script>(async () => {try {' + setup + source + checks + '''
} catch(e) {document.body.textContent='LAYOUT FAILED: '+e.stack;}})();</script>'''
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'layout.html'
            path.write_text(html)
            for width, height in ((1440,1000), (1366,768), (1024,768),
                                  (800,600), (500,900), (500,650), (900,550)):
                with self.subTest(viewport=(width,height)):
                    result = subprocess.run([
                        'chromium', '--headless', '--no-sandbox', '--disable-gpu',
                        f'--window-size={width},{height}', '--force-device-scale-factor=1',
                        f'--user-data-dir={tmp}/profile-{width}-{height}',
                        '--virtual-time-budget=1500', '--dump-dom', path.as_uri(),
                    ], capture_output=True, text=True, timeout=60)
                    failure = re.search(r'<body[^>]*>(LAYOUT FAILED:.*?)</body>', result.stdout, re.S)
                    self.assertTrue('LAYOUT PASSED</body>' in result.stdout,
                                    failure.group(1) if failure else result.stderr[-2000:])


if __name__ == '__main__':
    unittest.main()
