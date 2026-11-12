"""Exercise load-cell setup with the real panel and UI, without ROS or hardware."""
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
import unittest

DASHBOARD = Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'Chromium required')
class BrowserLoadCellTest(unittest.TestCase):
    def test_source_zero_threshold_graph_and_disconnection(self):
        html = (DASHBOARD / 'index.html').read_text().split('  <script>')[0]
        html = re.sub(r'<link rel="stylesheet" href="assets/([^\"]+)">', lambda match:
                      '<style>' + (DASHBOARD / match[1]).read_text() + '</style>', html)
        setup = '''
window.requestAnimationFrame = () => 0;
window.setInterval = () => 0;
window.setTimeout = () => 0;
window.sent = [];
window.WebSocket = class {static OPEN=1;readyState=1;send(value){window.sent.push(JSON.parse(value));}};
window.fetch = () => new Promise(() => {});
'''
        checks = '''
const assert = (value,label) => {if (!value) throw Error(label);};
state.connected=true;state.drive.you_control_owner=true;
state.settings={probe_contact:{source:'adc',threshold_counts:null,zero_counts:null}};
const frame = window.operatorContactSourceFrame;
const edit = (id,value) => {$(id).value=value;$(id).dispatchEvent(new Event(id==='contact-source'?'change':'input'));};
frame();
assert(!$('contact-source').disabled && $('contact-source-save').disabled,'saved ADC starts clean');
edit('contact-source','esp32');
assert(!$('contact-source-save').disabled && $('loadcell-threshold').value==='','select ESP32 without a guessed threshold');
assert(!$('loadcell-known-mass') && !$('loadcell-calibrate') && !$('loadcell-usb-serial') && !$('loadcell-hysteresis'),'no mass calibration or advanced options');
$('contact-source-save').click();
assert(sent.at(-1).type==='settings' && sent.at(-1).value.source==='esp32' && sent.at(-1).value.threshold_counts===null,'save selection before setting up sensor');
state.settings.probe_contact=structuredClone(sent.at(-1).value);
state.probe.loadcell={connected:true,fresh:true,raw_counts:1200000,net_counts:null,zeroed:false,busy:false,error:null,session:'first',sample_ms:200,
  samples:[{x:-.3,raw_counts:1200000},{x:-.2,raw_counts:1200100},{x:-.1,raw_counts:1199900}]};
state.probe.history_time_s=100;state.probe.history_age_s=0;
state.probe.adc_contact_state={source:'esp32',enabled:false,fresh:false,detected:null,level:0};frame();
assert(!$('loadcell-zero').disabled && $('contact-live-volts').textContent.includes('raw'),'read and zero immediately without threshold or calibration');
assert(operatorContactBlock().includes('Zero'),'probing still requires saved zero');
$('settings-dialog').showModal();
document.querySelectorAll('[data-settings-panel]').forEach(panel=>{panel.hidden=panel.id!=='settings-contact';panel.classList.toggle('active',panel.id==='settings-contact');});
const draw = () => {const previous=Chart.getChart('contact-history');if(previous) previous.$frameBucket=null;operatorADCFrame();return Chart.getChart('contact-history');};
let chart=draw();
assert(chart && chart.data.datasets[0].label.includes('raw counts') && chart.data.datasets[0].data.length===3,'raw graph works without ADC telemetry');
assert(chart.data.datasets[0].data[0].y===1200000 && chart.data.datasets.length===1,'graph does not invent a zero or threshold');
state.probe.loadcell.can_zero=false;frame();
assert($('loadcell-zero').disabled && $('loadcell-feedback').textContent.includes('10 continuous seconds'),'zero waits for ten seconds of readings');
state.probe.loadcell.can_zero=true;frame();
$('loadcell-zero').click();
assert(sent.at(-1).type==='loadcell' && sent.at(-1).action==='zero' && $('loadcell-zero').disabled,'one Zero command');
assert($('contact-source').disabled && $('loadcell-threshold').disabled && $('contact-source-save').disabled,'settings wait while zero is saved');
state.settings.probe_contact.zero_counts=1200000;state.probe.loadcell.zeroed=true;state.probe.loadcell.net_counts=0;
operatorContactSourceResult({type:'loadcell_result',ok:true,message:'Zero saved'});
assert(operatorContactBlock().includes('threshold'),'zero alone cannot enable probing');
chart=draw();
assert(chart.data.datasets[0].data[0].y===0 && chart.data.datasets[0].data[2].y===100,'graph shows absolute change from zero');
edit('loadcell-threshold','0');assert($('contact-source-save').disabled,'zero threshold rejected');
edit('loadcell-threshold','200');
state.settings.probe_contact.zero_counts=1200010;state.probe.loadcell.net_counts=10;frame();
assert($('loadcell-threshold').value==='200','saved zero update preserves threshold draft');
$('contact-source-save').click();
assert(Object.keys(sent.at(-1).value).length===3 && sent.at(-1).value.threshold_counts===200 && sent.at(-1).value.zero_counts===1200010,'save only source, threshold and current zero');
state.settings.probe_contact=structuredClone(sent.at(-1).value);
state.probe.adc_contact_state={source:'esp32',enabled:true,fresh:true,detected:false,level:200};frame();
assert(operatorContactBlock()==='','ESP32 contact works without ADC');
chart=draw();
assert(chart.data.datasets.length===2 && chart.data.datasets[1].data[0].y===200,'one positive threshold matches magnitude trigger');
assert(chart.data.datasets[0].data[2].y===110,'history uses saved zero and preserves either force direction');
frame();chart=draw();assert(chart.data.datasets[0].data.length===3,'duplicate telemetry does not append samples');
state.probe.loadcell.samples=[{x:-1.2,raw_counts:1200000},{x:-.1,raw_counts:1200010}];
state.probe.history_time_s=101;frame();chart=draw();
assert(chart.data.datasets[0].data[1].y===null,'missing sample interval stays a graph gap');
state.probe.pressure_ratio=.2;updateHud();
for(const id of ['probe-map-sensor','probe-adc-contact','adc-trigger-state']) assert($(id).textContent.includes('10 counts') && !$(id).textContent.includes('-10') && $(id).textContent.includes('ESP32'),'main readout uses magnitude counts: '+id);
state.probe.loadcell.filtered_counts=1200030;state.probe.loadcell.net_counts=20;
state.probe.loadcell.samples=[{x:-.3,raw_counts:1200200,filtered_counts:1200050},{x:-.2,raw_counts:1199000,filtered_counts:1199970},{x:-.1,raw_counts:1200010}];
state.probe.history_time_s=102;frame();chart=draw();
assert(chart.data.datasets[0].data[0].y===40 && chart.data.datasets[0].data[1].y===40 && chart.data.datasets[0].data[2].y===0,'filtered graph uses magnitude with legacy fallback');
assert(chart.data.datasets[0].label.includes('filtered') && $('contact-live-volts').textContent.includes('filtered'),'filtered signal is identified');
assert($('probe-pressure').textContent==='20.0%' && $('load-heading').textContent==='Motor load','motor diagnostic stays separate');
state.probe.adc_contact_state.fresh=false;assert(operatorContactBlock(),'stale contact evaluation blocks probing');state.probe.adc_contact_state.fresh=true;
state.probe.cycle_active=true;frame();assert($('loadcell-zero').disabled,'cannot zero during probe cycle');state.probe.cycle_active=false;
state.probe.loadcell.fresh=false;frame();chart=draw();
assert($('contact-live-volts').textContent==='—' && $('loadcell-zero').disabled && !$('contact-draft-preview').hidden,'stale readings blank and graph identifies prior samples');
state.probe.loadcell.fresh=true;state.drive.you_control_owner=false;frame();
assert($('contact-source').disabled && $('loadcell-zero').disabled,'spectators cannot change sensor setup');
state.drive.you_control_owner=true;state.connected=false;frame();chart=draw();
assert($('contact-live-volts').textContent==='—' && $('loadcell-zero').disabled && chart.data.datasets[0].data.length===0,'disconnection removes current reading and plot');
document.body.textContent='LOADCELL UI PASSED';
'''
        source = '\n'.join((DASHBOARD / name).read_text() for name in ('chart-4.4.8.umd.js','keydrown-1.3.0.js','actuator-settings.js','app.js','adc.js','camera-config.js'))
        html += '<script>(async()=>{try{' + setup + source + checks + "}catch(e){document.body.textContent='LOADCELL UI FAILED: '+e.stack;}})();</script>"
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'loadcell.html'
            path.write_text(html)
            result = subprocess.run(['chromium','--headless','--no-sandbox','--disable-gpu',
                '--user-data-dir='+tmp+'/profile','--virtual-time-budget=1000','--dump-dom',path.as_uri()],
                capture_output=True,text=True,timeout=60)
            body = re.sub(r'<head>.*?</head>', '', result.stdout, flags=re.S)
            self.assertIn('LOADCELL UI PASSED</body>',body,body+result.stderr[-1000:])


if __name__ == '__main__':
    unittest.main()
