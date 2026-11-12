/* ADS1115 settings and batched telemetry. Acquisition belongs to the gateway. */
(() => {
  const el = id => document.getElementById(`adc-${id}`);
  const names = ['A0 − A1', 'A0 − A3', 'A1 − A3', 'A2 − A3', 'A0', 'A1', 'A2', 'A3'];
  const colors = ['#d896ef', '#f5ac72', '#8ca8ff', '#ec8fac', '#66dbbd', '#efcf70', '#73c9ff', '#bac888'];
  const ranges = [6.144, 4.096, 2.048, 1.024, .512, .256];
  const fields = [...document.querySelectorAll('[data-adc-field]')];
  const field = key => fields.find(input => input.dataset.adcField === key);
  field('rate').innerHTML = [8,16,32,64,128,250,475,860].map(n => `<option value="${n}">${n} SPS</option>`).join('');
  field('pga').innerHTML = ranges.map((n,i) => `<option value="${i}">±${n} V</option>`).join('');
  el('input-settings').innerHTML = names.map((name,i) => `<tr data-adc-channel="${i}"><td><input type="checkbox" aria-label="Enable ${name}"></td><td>${name}</td><td><input type="number" step="any" aria-label="${name} multiplier"></td><td><input type="number" step="any" aria-label="${name} offset"></td><td>—</td></tr>`).join('');
  const rows = [...document.querySelectorAll('[data-adc-channel]')];
  const inputOptions = names.map((name,i) => `<option value="${i}">${name}</option>`).join('');
  const muxInput = el('trigger-mux'), enabledInput = el('trigger-enabled');
  const hysteresis = el('trigger-hysteresis'), debounce = el('trigger-debounce');
  const detectorInput = document.getElementById('metal-adc-input');
  const detectorSave = document.getElementById('metal-adc-save');
  const smoothingInput = document.getElementById('metal-smoothing');
  const smoothingSave = document.getElementById('metal-smoothing-save');
  let smoothingError = '';
  const detectorFeedback = document.getElementById('metal-adc-feedback');
  muxInput.innerHTML = inputOptions;
  detectorInput.innerHTML = '<option value="">Unassigned</option>' + inputOptions;
  let latest = null, revision = null, session = null, cursor = 0, history = [], chart = null, clockOffset = 0;
  let pending = false, importedSettings = null, adcOnline = false;
  let contactChart = null, detectorKey = '', detectorDirty = false, detectorMessage = '';
  const contactDirection = document.getElementById('probe-contact-direction');
  const contactVolts = document.getElementById('probe-contact-volts');
  const contactSave = document.getElementById('probe-contact-save');
  const contactFeedback = document.getElementById('probe-contact-feedback');
  let contactDraftKey = '', contactDirty = false, contactMessage = '';
  const opposite = direction => direction === 'above' ? 'below' : 'above';
  const detectorRole = role => role === 'detector' || role === 'both';
  const assignedRoutes = (detectors, probeMux) => names.map((_, mux) => detectors.includes(mux)
    ? (mux === probeMux ? 'both' : 'detector') : (mux === probeMux ? 'probe' : 'none'));
  const detectorMuxes = settings => settings.routes.flatMap((role, mux) => detectorRole(role) ? [mux] : []);
  const channelAt = mux => latest?.config.channels.find(channel => channel.mux === mux);
  function unscale(value, channel) {
    if (!Number.isFinite(value) || !Number.isFinite(channel?.multiplier) || channel.multiplier === 0 || !Number.isFinite(channel.offset)) return null;
    const volts = (value - channel.offset) / channel.multiplier;
    return Number.isFinite(volts) ? volts : null;
  }
  function inputProblem(mux, needsEnabled = true) {
    const channel = channelAt(mux);
    if (!channel) return 'Waiting for ADC settings.';
    if (needsEnabled && !channel.enabled) return `Enable ${names[mux]} in Setup → Sensor board.`;
    if (!Number.isFinite(channel.multiplier) || channel.multiplier === 0 || !Number.isFinite(channel.offset))
      return 'Set a finite offset and a nonzero input multiplier in Setup → Sensor board.';
    return '';
  }
  const sourceInput = document.getElementById('contact-source');
  const sourceSave = document.getElementById('contact-source-save');
  const sourceFeedback = document.getElementById('contact-source-feedback');
  const cellThreshold = document.getElementById('loadcell-threshold');
  const cellZero = document.getElementById('loadcell-zero');
  let sourceDirty = false, sourcePending = null, sourceMessage = '', cellPending = null, cellMessage = '';
  let cellHistory = [], cellHistorySession = null, cellHistoryStamp = null, cellHistoryAt = 0;
  function sourceDraft() {
    return {source:sourceInput.value, threshold_counts:cellThreshold.value === '' ? null : cellThreshold.valueAsNumber};
  }
  function sourceChanged() {
    const saved = state.settings?.probe_contact;
    return Boolean(saved && Object.entries(sourceDraft()).some(([key,value]) => saved[key] !== value));
  }
  function sourceProblem() {
    const draft = sourceDraft();
    if (!['adc','esp32'].includes(draft.source)) return 'Choose a contact sensor.';
    if (draft.threshold_counts !== null && (!Number.isFinite(draft.threshold_counts) || draft.threshold_counts <= 0)) return 'Enter a positive threshold or leave it blank while setting up.';
    return '';
  }
  function loadcellProblem() {
    const cell = state.probe?.loadcell, saved = state.settings?.probe_contact;
    if (!state.connected || !cell?.connected) return cell?.error || 'Waiting for the ESP32 load cell.';
    if (cell.busy || cellPending) return 'Waiting for the zero to save.';
    if (!cell.fresh) return cell.error || 'Waiting for a fresh load-cell reading.';
    if (!cell.zeroed || !Number.isFinite(saved?.zero_counts)) return 'Zero the unloaded probe in Setup → Probe contact.';
    if (!Number.isFinite(saved.threshold_counts) || saved.threshold_counts <= 0) return 'Set a contact threshold in Setup → Probe contact.';
    return '';
  }
  window.operatorContactBlock = () => sourcePending || (sourceDirty && sourceChanged()) ? 'Save or undo your edits in Setup → Probe contact.'
    : state.settings?.probe_contact?.source === 'esp32' ? loadcellProblem() || (state.probe?.adc_contact_state?.source !== 'esp32' || state.probe.adc_contact_state.level !== state.settings.probe_contact.threshold_counts || !state.probe.adc_contact_state.enabled || !state.probe.adc_contact_state.fresh ? 'Waiting for the saved contact threshold to apply.' : '')
    : !adcOnline || !latest?.probe_trigger ? 'Waiting for contact settings.'
    : inputProblem(latest.probe_trigger.mux) || (pending ? 'Wait for ADC settings to finish saving.'
    : contactDirty ? 'Save or undo your edits in Setup → Probe contact.' : '');

  function renderContactSource() {
    const saved = state.settings?.probe_contact;
    const owner = state.connected && state.drive.you_control_owner;
    if (!owner) { sourceDirty = false; sourcePending = null; sourceMessage = ''; cellPending = null; }
    if (sourcePending) {
      if (saved && Object.entries(sourcePending.value).every(([key,value]) => saved[key] === value)) {
        sourcePending = null; sourceDirty = false; sourceMessage = 'Saved on car.';
      } else if (performance.now() - sourcePending.at > 5000) {
        sourcePending = null; sourceMessage = 'No save confirmation. Check the car settings and retry.';
      }
    }
    if (!sourceDirty && saved) {
      sourceInput.value = saved.source;
      cellThreshold.value = saved.threshold_counts ?? '';
    }
    const esp32 = sourceInput.value === 'esp32';
    document.querySelectorAll('[data-contact-adc]').forEach(card => { card.hidden = esp32; });
    document.querySelectorAll('[data-contact-loadcell]').forEach(card => { card.hidden = !esp32; });
    const problem = saved ? sourceProblem() : '';
    const cell = state.probe?.loadcell;
    const editable = owner && Boolean(saved) && !sourcePending && !cellPending && !cell?.busy;
    for (const input of [sourceInput,cellThreshold]) input.disabled = !editable;
    sourceSave.disabled = !editable || Boolean(problem) || !sourceChanged();
    document.getElementById('contact-source-revert').disabled = !editable || !sourceDirty;
    sourceFeedback.textContent = !saved ? 'Waiting for car settings.' : !owner ? 'Take control to edit.'
      : sourcePending ? 'Saving…' : sourceMessage || problem || (sourceChanged() ? 'Unsaved contact settings.' : 'Saved on car.');
    if (cell?.session !== cellHistorySession) {
      cellHistory = []; cellHistoryStamp = null; cellHistorySession = cell?.session;
    }
    if (Array.isArray(cell?.samples) && state.probe.history_time_s !== cellHistoryStamp) {
      cellHistory = cell.samples; cellHistoryStamp = state.probe.history_time_s;
      cellHistoryAt = performance.now() - Math.max(0,state.probe.history_age_s ?? 0)*1000;
    }
    const live = state.connected && cell?.connected && cell.fresh && !cell.busy;
    const zeroed = cell?.zeroed && Number.isFinite(saved?.zero_counts);
    const filtered = Number.isFinite(cell?.filtered_counts);
    const value = zeroed ? cell?.net_counts : filtered ? cell.filtered_counts : cell?.raw_counts;
    const reading = live && Number.isFinite(value) ? `${Math.round(value)} counts · ${filtered ? 'filtered' : 'raw'}${zeroed ? ' · magnitude from zero' : ''}` : '—';
    const contact = state.probe?.adc_contact_state;
    const cellStatus = !live ? cell?.error || 'Waiting for the ESP32 load cell.'
      : `${contact?.source === 'esp32' && contact.fresh ? (contact.detected === true ? 'CONTACT' : 'Clear') : zeroed ? 'Live' : 'Zero needed'} · ${reading}`;
    if (esp32) {
      document.getElementById('contact-live-volts').textContent = reading;
      const preview = document.getElementById('contact-draft-preview');
      preview.hidden = live || !cellHistory.length;
      preview.textContent = 'No fresh reading · previous samples only';
    }
    if (saved?.source === 'esp32') {
      for (const id of ['adc-trigger-state','probe-adc-contact','probe-map-sensor']) document.getElementById(id).textContent = `${cellStatus} · ESP32`;
    }
    if (cellPending && performance.now() - cellPending.at > 5000) { cellPending = null; cellMessage = 'No zero confirmation. Check the connection and retry.'; }
    const moving = state.probe?.active || state.probe?.moving || state.probe?.homing || state.probe?.cycle_active || state.actuators?.probe?.homing || state.actuators?.probe?.torque;
    cellZero.disabled = !owner || saved?.source !== 'esp32' || !live || !Number.isFinite(cell.raw_counts) || cell.can_zero === false || moving || Boolean(cellPending) || Boolean(sourcePending) || sourceChanged();
    document.getElementById('loadcell-feedback').textContent = cellPending || cell?.busy ? 'Saving zero on car…'
      : moving ? 'Stop the probe and release torque before zeroing.' : live && cell.can_zero === false ? 'Keep the probe unloaded for 10 continuous seconds before zeroing.'
      : cellMessage || (live && !zeroed ? 'Lift the probe clear, then zero it.' : live && !saved?.threshold_counts ? 'Press the sensor and choose a threshold from the graph.' : '');
    document.getElementById('contact-history').setAttribute('aria-label', esp32 ? 'Load-cell counts, shown as magnitude from zero after zeroing, and contact threshold over the last 30 seconds' : 'Filtered contact sensor voltage and contact limit over the last 30 seconds');
  }
  window.operatorContactSourceFrame = renderContactSource;
  const editSource = () => { sourceDirty = true; sourceMessage = ''; renderContactSource(); if (contactChart) contactChart.$frameBucket = null; window.operatorADCFrame?.(); };
  sourceInput.onchange = editSource;
  cellThreshold.oninput = editSource;
  document.getElementById('contact-source-revert').onclick = () => { sourceDirty=false; sourceMessage=''; renderContactSource(); };
  sourceSave.onclick = () => {
    if (sourceSave.disabled || sourceProblem()) return;
    const saved = state.settings.probe_contact;
    const value = {...sourceDraft(),zero_counts:saved.zero_counts};
    sourcePending = {value:sourceDraft(),at:performance.now()}; sourceMessage = '';
    send({type:'settings',section:'probe_contact',value}); renderContactSource();
  };
  cellZero.onclick = () => {
    if (cellZero.disabled) return;
    cellPending = {at:performance.now()}; cellMessage = '';
    send({type:'loadcell',action:'zero'}); renderContactSource();
  };
  window.operatorContactSourceResult = reply => {
    if (reply.type === 'loadcell_result') { cellPending=null; cellMessage=reply.message || (reply.ok ? 'Zero saved on car.' : 'Zero could not be saved.'); }
    else if (reply.section === 'probe_contact' && !reply.ok) { sourcePending=null; sourceMessage=reply.message || 'Save rejected.'; }
    renderContactSource();
  };

  function renderSmoothing() {
    const filter = field('filter_ms');
    if (smoothingInput.value !== filter.value) smoothingInput.value = filter.value;
    const applied = latest?.config.filter_ms;
    const draft = smoothingInput.valueAsNumber;
    const valid = smoothingInput.validity.valid;
    const dirty = valid && draft !== applied;
    const running = latest?.connected && latest.running && latest.applied === latest.revision;
    smoothingSave.disabled = !adcOnline || !state.connected || !state.drive.you_control_owner || Boolean(pending) || !dirty;
    document.getElementById('metal-smoothing-feedback').textContent = !adcOnline ? 'Waiting for ADC settings.'
      : pending === 'smoothing' ? 'Saving smoothing…' : smoothingError
      || (!valid ? 'Enter a whole number from 0 to 2000 ms.'
        : `${running ? 'Applied' : 'Configured'}: ${applied} ms${dirty ? ' · Preview: ' + draft + ' ms' : ''}`);
  }
  smoothingInput.oninput = () => {
    field('filter_ms').value = smoothingInput.value;
    smoothingError = '';
    renderSmoothing();
    drawMetalHistory();
  };
  smoothingSave.onclick = () => {
    if (smoothingSave.disabled || !smoothingInput.reportValidity()) return;
    const settings = savedSettings();
    settings.config.filter_ms = smoothingInput.valueAsNumber;
    command({action:'settings', settings}, 'smoothing');
    renderSmoothing();
  };
  function hysteresisProblem(mux, level, width, above, pga) {
    if (!(width > 0) || !Number.isFinite(level)) return '';
    const full = ranges[pga], low = mux >= 4 ? 0 : -full, high = full*32767/32768;
    const room = Math.min(high-low, above ? level-low : high-level);
    return width < room ? '' : (room > 0
      ? `Hysteresis must be below ${Number(room.toPrecision(6))} V for this input to clear.`
      : 'Use 0 V hysteresis or move the limit inside the input range.');
  }
  function renderSensors() {
    renderContactSource();
    renderSmoothing();
    const trigger = latest?.probe_trigger;
    const owner = adcOnline && state.connected && state.drive.you_control_owner;
    if (!trigger) return;
    const channel = channelAt(trigger.mux);
    const key = JSON.stringify([trigger, channel?.multiplier, channel?.offset]);
    if (!contactDirty && key !== contactDraftKey && channel) {
      muxInput.value = trigger.mux;
      enabledInput.checked = trigger.enabled;
      const levelVolts = unscale(trigger.level, channel);
      contactVolts.value = levelVolts === null ? '' : Number(levelVolts.toFixed(9));
      contactDirection.value = channel.multiplier < 0 ? opposite(trigger.direction) : trigger.direction;
      hysteresis.value = channel.multiplier ? Number(((trigger.hysteresis || 0) / Math.abs(channel.multiplier)).toFixed(9)) : 0;
      debounce.value = trigger.debounce_ms || 0;
      contactDraftKey = key;
    }
    const widthProblem = hysteresisProblem(Number(muxInput.value), contactVolts.valueAsNumber,
      hysteresis.valueAsNumber, contactDirection.value === 'above', latest.config.pga);
    hysteresis.setCustomValidity(widthProblem);
    const issue = inputProblem(Number(muxInput.value), enabledInput.checked) || widthProblem;
    const disabling = trigger.enabled && !enabledInput.checked && Number(muxInput.value) === trigger.mux;
    contactSave.disabled = !owner || Boolean(pending) || Boolean(issue && !disabling);
    for (const input of [muxInput, enabledInput, contactVolts, contactDirection, hysteresis, debounce]) input.disabled = Boolean(pending);
    const contact = latest.probe_contact;
    const reading = contact && Object.hasOwn(contact, 'filtered_volts') ? contact.filtered_volts : unscale(contact?.value, channel);
    const volts = adcOnline && latest.connected && latest.running && latest.applied === latest.revision && contact?.fresh && contact.mux === trigger.mux && Number.isFinite(reading) ? reading : null;
    const status = !adcOnline ? 'ADC disconnected' : !trigger.enabled ? 'Contact detection disabled'
      : volts === null ? 'Waiting for contact reading.' : `${contact.detected ? 'CONTACT' : 'Clear'} · ${names[trigger.mux]} · ${volts.toFixed(4)} V`;
    document.getElementById('probe-adc-contact').textContent = status;
    document.getElementById('probe-map-sensor').textContent = status;
    const limit = unscale(trigger.level, channel);
    const width = channel?.multiplier ? (trigger.hysteresis || 0)/Math.abs(channel.multiplier) : null;
    const above = channel?.multiplier < 0 ? trigger.direction === 'below' : trigger.direction === 'above';
    const levels = trigger.enabled && Number.isFinite(limit) && Number.isFinite(width)
      ? ` · contact ${above ? '≥' : '≤'} ${limit.toFixed(4)} V · clear ${above ? '<' : '>'} ${(above ? limit-width : limit+width).toFixed(4)} V · ${trigger.debounce_ms || 0} ms debounce` : '';
    el('trigger-state').textContent = `Applied · ${status}${levels}`;
    renderContactSignal();
    contactFeedback.textContent = (issue && disabling ? 'Save to disable contact detection.' : issue) || (pending === 'contact' ? 'Saving contact settings…' : contactMessage) || latest.contact_warning
      || (!owner ? 'Take control to save contact settings.' : contactDirty ? 'Unsaved contact settings' : '');
    const detectors = detectorMuxes(latest);
    const active = detectors.filter(mux => channelAt(mux)?.enabled);
    const detectorSignature = JSON.stringify([detectors, active]);
    if (!detectorDirty && detectorKey !== detectorSignature) {
      detectorInput.value = active.length === 1 ? active[0] : detectors.length === 1 ? detectors[0] : '';
      detectorKey = detectorSignature;
    }
    const detectorIssue = detectorInput.value !== '' && !channelAt(Number(detectorInput.value))?.enabled
      ? `Enable ${names[Number(detectorInput.value)]} in Setup → Sensor board.` : '';
    detectorSave.disabled = !owner || Boolean(pending) || Boolean(detectorIssue);
    detectorFeedback.textContent = detectorIssue || (pending === 'detector' ? 'Saving sensor input…' : detectorMessage)
      || (!detectorDirty && detectors.length > 1 ? 'Multiple saved detector inputs. Select and save one input.' : '');
    const routes = assignedRoutes(detectors, trigger.mux);
    rows.forEach((row, mux) => {
      const label = {none:'—', detector:'Metal detector', probe:'Probe contact', both:'Metal detector + probe contact'}[routes[mux]];
      if (row.lastElementChild.textContent !== label) row.lastElementChild.textContent = label;
    });
    renderContactSource();
  }
  function savedSettings() {
    return {config: structuredClone(latest.config), routes:[...latest.routes], probe_trigger:{...latest.probe_trigger}};
  }
  const editContact = () => { contactDirty = true; contactMessage = ''; renderSensors(); };
  for (const input of [contactVolts, hysteresis, debounce]) input.oninput = editContact;
  for (const input of [contactDirection, muxInput, enabledInput]) input.onchange = editContact;
  document.getElementById('contact-revert').onclick = () => {
    contactDirty = false; contactDraftKey = ''; contactMessage = ''; renderSensors();
  };
  contactSave.onclick = () => {
    if (contactSave.disabled) return;
    const mux = Number(muxInput.value), channel = channelAt(mux);
    const settings = savedSettings();
    const fields = [contactVolts, hysteresis, debounce];
    // Disabling the applied input must still work with a bad draft or legacy gain.
    // Retain the applied limit and discard those invalid edits.
    if (!enabledInput.checked && mux === settings.probe_trigger.mux &&
        (!channel.multiplier || fields.some(input => !input.validity.valid))) {
      settings.probe_trigger.enabled = false;
      const saved = settings.probe_trigger;
      if (!channel.multiplier || hysteresisProblem(mux, unscale(saved.level, channel),
          (saved.hysteresis || 0)/Math.abs(channel.multiplier),
          channel.multiplier < 0 ? saved.direction === 'below' : saved.direction === 'above', settings.config.pga))
        saved.hysteresis = 0;
    } else {
      if (!fields.every(input => input.reportValidity())) return;
      settings.probe_trigger = {...settings.probe_trigger, enabled:enabledInput.checked, mux,
        level:contactVolts.valueAsNumber * channel.multiplier + channel.offset,
        direction:channel.multiplier < 0 ? opposite(contactDirection.value) : contactDirection.value,
        hysteresis:hysteresis.valueAsNumber * Math.abs(channel.multiplier), debounce_ms:debounce.valueAsNumber};
    }
    settings.routes = assignedRoutes(detectorMuxes(latest), mux);
    contactMessage = '';
    command({action:'settings', settings}, 'contact');
    renderSensors();
  };
  detectorInput.onchange = () => { detectorDirty = true; detectorMessage = ''; renderSensors(); };
  detectorSave.onclick = () => {
    if (detectorSave.disabled) return;
    const settings = savedSettings();
    settings.routes = assignedRoutes(detectorInput.value === '' ? [] : [Number(detectorInput.value)], settings.probe_trigger.mux);
    command({action:'settings', settings}, 'detector');
    renderSensors();
  };

  const isThreshold = key => key === 'low' || key === 'high';
  const thresholdVolts = (counts, pga) => Number((counts * ranges[pga] / 32768).toFixed(10));
  const thresholdCounts = (volts, pga) => Math.sign(volts) * Math.round(Math.abs(volts) * 32768 / ranges[pga]);

  function fill(value) {
    const {pga, low, high} = value.config;
    if (!Number.isInteger(pga) || !ranges[pga] || ![low, high].every(n => Number.isInteger(n) && n >= -32768 && n <= 32767))
      throw Error('Invalid ADC input range or comparator register values.');
    fields.forEach(input => {
      const key = input.dataset.adcField;
      const v = isThreshold(key) ? thresholdVolts(value.config[key], pga) : value.config[key];
      if (input.type === 'checkbox') input.checked = v; else input.value = v;
    });
    rows.forEach((row,i) => {
      const inputs = row.querySelectorAll('input');
      inputs[0].checked = value.config.channels[i].enabled;
      inputs[1].value = value.config.channels[i].multiplier;
      inputs[2].value = value.config.channels[i].offset;
    });
    equivalents();
  }
  function draft() {
    const config = {};
    equivalents();
    fields.forEach(input => {
      const key = input.dataset.adcField;
      if (isThreshold(key)) {
        if (!input.checkValidity()) throw Error(`Enter the ${key} threshold between ${input.min} and ${input.max} V.`);
        config[key] = thresholdCounts(input.valueAsNumber, Number(field('pga').value));
      } else {
        if (input.type === 'number' && (!input.value || !input.checkValidity())) throw Error(`Invalid ${key}`);
        config[key] = input.type === 'checkbox' ? input.checked : ['mode','alert'].includes(key) ? input.value : Number(input.value);
      }
    });
    if (['traditional','window'].includes(config.alert) && config.low >= config.high)
      throw Error('The rounded low threshold must be below the rounded high threshold.');
    config.channels = rows.map((row,mux) => {
      const inputs = row.querySelectorAll('input');
      if ([...inputs].slice(1).some(input => !input.value || !input.checkValidity())) throw Error('Enter finite scaling values');
      return {mux, enabled: inputs[0].checked, multiplier: Number(inputs[1].value), offset: Number(inputs[2].value)};
    });
    const source = importedSettings || latest;
    if (!source) throw Error('Waiting for ADC settings.');
    const trigger = {...source.probe_trigger};
    const previous = source.config.channels[trigger.mux], next = config.channels[trigger.mux];
    if (previous.multiplier !== next.multiplier || previous.offset !== next.offset) {
      if (next.multiplier === 0) throw Error('Use a nonzero multiplier for the probe contact input.');
      if (previous.multiplier === 0) {
        if (trigger.enabled) throw Error('Disable detection in Probe contact before correcting its zero multiplier.');
        // There was no physical threshold to carry over. Detection stays off.
      } else {
        const volts = unscale(trigger.level, previous);
        if (volts === null) throw Error('The current probe limit cannot be converted to voltage.');
        trigger.level = volts * next.multiplier + next.offset;
        trigger.hysteresis = (trigger.hysteresis || 0) / Math.abs(previous.multiplier) * Math.abs(next.multiplier);
        if (![trigger.level, trigger.hysteresis].every(n => Number.isFinite(n) && Math.abs(n) <= 1e12))
          throw Error('The new scaling cannot represent the saved probe contact settings.');
        if (Math.sign(previous.multiplier) !== Math.sign(next.multiplier)) trigger.direction = opposite(trigger.direction);
      }
    }
    return {config, routes:[...source.routes], probe_trigger:trigger};
  }
  function equivalents() {
    const pga = Number(field('pga').value);
    const min = thresholdVolts(-32768, pga), max = thresholdVolts(32767, pga);
    el('resolution').textContent = `${(ranges[pga] / 32768 * 1e6).toFixed(3)} µV / count`;
    for (const key of ['low','high']) {
      const input = field(key), volts = input.valueAsNumber;
      input.min = min;
      input.max = max;
      const rounded = thresholdVolts(thresholdCounts(volts, pga), pga);
      el(`${key}-volts`).textContent = !input.checkValidity() ? `Range ${min} to ${max} V`
        : rounded !== volts ? `Nearest setting: ${rounded} V` : '';
    }
  }
  fields.forEach(input => input.addEventListener('input', equivalents));
  function command(value, source = 'board') {
    if (!state.connected || !state.drive.you_control_owner) { el('feedback').textContent = 'Take control before changing ADC settings.'; return; }
    if (pending || !adcOnline) return;
    pending = source;
    // A board range change or imported file can also alter the reachable release voltage.
    const trigger = value.settings?.probe_trigger;
    if (trigger) {
      const config = value.settings.config, channel = config.channels[trigger.mux];
      const above = channel.multiplier < 0 ? trigger.direction === 'below' : trigger.direction === 'above';
      const issue = hysteresisProblem(trigger.mux, unscale(trigger.level, channel),
        (trigger.hysteresis || 0)/Math.abs(channel.multiplier), above, config.pga);
      if (issue) { window.operatorADCResult({ok:false, message:issue}); return; }
    }
    el('apply').disabled = true;
    if (source === 'board') el('feedback').textContent = 'Saving…';
    send({type:'adc', ...value, expected_adc_revision:latest?.saved_config_revision});
  }
  window.operatorADCResult = message => {
    const source = pending;
    pending = false;
    if (source === 'contact') {
      if (message.ok) contactDirty = false;
      contactMessage = message.ok ? 'Contact settings saved.' : message.message;
    } else if (source === 'smoothing') {
      smoothingError = message.ok ? '' : message.message || 'Smoothing could not be saved.';
    } else if (source === 'detector') {
      if (message.ok) detectorDirty = false;
      detectorMessage = message.ok ? 'Sensor input saved.' : message.message;
    } else {
      el('feedback').textContent = message.message || (message.ok ? 'Saved' : 'Save failed');
      if (message.ok) { importedSettings = null; smoothingError = ''; }
    }
    render();
  };
  el('apply').onclick = () => { try { command({action:'settings', settings:draft()}); } catch (error) { el('feedback').textContent = error.message; } };
  el('revert').onclick = () => { if (latest) { importedSettings = null; fill(latest); } };
  function download(text, type, name) {
    const url = URL.createObjectURL(new Blob([text], {type}));
    const a = document.createElement('a'); a.href = url; a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  el('save').onclick = () => { try { download(JSON.stringify(draft(),null,2), 'application/json', 'ads1115.json'); } catch (error) { el('feedback').textContent = error.message; } };
  el('import').onclick = () => el('file').click();
  el('file').onchange = async () => {
    try {
      const value = JSON.parse(await el('file').files[0].text());
      const config = {filter_ms:100, ...(value.config || value)}; // Also accept the standalone bench export.
      const routes = value.routes || latest?.routes || Array(8).fill('none');
      if (!config.channels || config.channels.length !== 8 || routes.length !== 8 || fields.some(f => !(f.dataset.adcField in config))) throw Error('Invalid ADC configuration');
      if (value.probe_trigger && (!Number.isFinite(value.probe_trigger.level) || Math.abs(value.probe_trigger.level) > 1e12 || !['above','below'].includes(value.probe_trigger.direction))) throw Error('Invalid imported contact limit');
      let trigger = value.probe_trigger || latest?.probe_trigger;
      if (!value.probe_trigger && value.routes) {
        const probeInputs = routes.flatMap((role, mux) => ['probe','both'].includes(role) ? [mux] : []);
        trigger = {enabled:false, mux:probeInputs.length === 1 ? probeInputs[0] : 4, level:1, direction:'above'};
      }
      if (!trigger) throw Error('Waiting for contact settings.');
      if (!Number.isInteger(trigger.mux) || trigger.mux < 0 || trigger.mux >= names.length) throw Error('Invalid imported probe input.');
      fill({config});
      importedSettings = {config, routes, probe_trigger:trigger};
      el('feedback').textContent = 'Imported draft. Apply & save to use it.';
    } catch (error) { el('feedback').textContent = error.message; }
    el('file').value = '';
  };
  el('csv').onclick = () => {
    const compare = el('quantity').value === 'comparison';
    const lines = [compare ? 'time,channel,filtered_volts,unfiltered_volts' : 'time,channel,filtered_volts'];
    for (const sample of history) {
      const row = [sample.time, names[sample.mux], sample.filtered_volts];
      if (compare) row.push(sample.volts);
      lines.push(row.join(','));
    }
    download(lines.join('\n'), 'text/csv', 'ads1115.csv');
  };
  const now = () => Date.now()/1000 + clockOffset;
  function datasets(target, quantity, seconds, selectedMux = null, time = now()) {
    if (!latest) return [];
    return latest.config.channels.filter(ch => ch.enabled && (selectedMux === null || ch.mux === selectedMux) && (!target || [target,'both'].includes(latest.routes[ch.mux]))).map(ch => {
      const samples = history.filter(s => s.mux === ch.mux && time-s.time <= seconds);
      const data = [];
      const gap = Math.max(.5, latest.config.interval_ms/1000 + 3 * latest.config.channels.filter(c => c.enabled).length/latest.config.rate);
      samples.forEach((s,i) => {
        if (i && s.time-samples[i-1].time > gap) data.push({x:s.time-time-.001, y:null});
        data.push({x:s.time-time,y:Number.isFinite(s[quantity]) ? s[quantity] : null});
      });
      return {label:`ADS ${names[ch.mux]} · ${quantity === 'filtered_volts' ? 'filtered' : 'unfiltered'} (V)`, yAxisID:'ads1115', data, borderColor:colors[ch.mux], borderWidth:1.5, pointRadius:0, spanGaps:false};
    });
  }
  const scaleBuckets = new Map();
  const scaleFields = ['volts', 'filtered_volts'];
  function rememberScale(samples, end) {
    for (const sample of samples) {
      const second = Math.floor(sample.time);
      let channels = scaleBuckets.get(second);
      if (!channels) { channels = new Map(); scaleBuckets.set(second, channels); }
      let range = channels.get(sample.mux);
      if (!range) { range = {}; channels.set(sample.mux, range); }
      for (const field of scaleFields) {
        const value = sample[field];
        if (!Number.isFinite(value)) continue;
        const bounds = range[field];
        if (bounds) {
          if (value < bounds.min) bounds.min = value;
          if (value > bounds.max) bounds.max = value;
        } else range[field] = {min: value, max: value};
      }
    }
    for (const second of scaleBuckets.keys()) if (end - second > SIGNAL_LOOKBACK_S) scaleBuckets.delete(second);
  }

  function adcAxis(route, quantity, threshold = null, selectedMux = null) {
    if (!latest) return {};
    let low = threshold ?? Infinity, high = threshold ?? -Infinity;
    const time = now(), voltsSpan = ranges[latest.config.pga] * .01;
    const quantities = quantity === 'comparison' ? ['volts', 'filtered_volts'] : [quantity];
    for (const channel of latest.config.channels) {
      if (!channel.enabled || (selectedMux !== null && channel.mux !== selectedMux) || (route && ![route, 'both'].includes(latest.routes[channel.mux]))) continue;
      for (const [second, channels] of scaleBuckets) {
        if (time - second > SIGNAL_LOOKBACK_S) continue;
        const channelBounds = channels.get(channel.mux);
        for (const field of quantities) {
          const bounds = channelBounds?.[field];
          if (bounds) { low = Math.min(low, bounds.min); high = Math.max(high, bounds.max); }
        }
      }
    }
    return signalBounds(low, high, voltsSpan);
  }

  window.operatorProbeVoltageHistory = () => {
    if (state.settings?.probe_contact?.source === 'esp32') return {data:[],limit:null,status:'ESP32 load cell selected; view counts in Setup → Probe contact.'};
    const trigger = latest?.probe_trigger;
    if (!trigger) return {data:[], limit:null, status:'Contact settings unavailable.'};
    const channel = channelAt(trigger.mux);
    const limit = unscale(trigger.level, channel);
    const time = now();
    const data = datasets(null, 'filtered_volts', 30, trigger.mux, time)[0]?.data || [];
    const gap = Math.max(.5, latest.config.interval_ms/1000 + 3*latest.config.channels.filter(c => c.enabled).length/latest.config.rate);
    const last = data.at(-1);
    const online = adcOnline && latest.connected && latest.running && latest.applied === latest.revision;
    let status = '';
    if (!online) status = 'Contact sensor offline.';
    else if (!channel?.enabled) status = 'Contact input disabled.';
    else if (!last || !Number.isFinite(last.y) || last.x < -gap) status = 'Contact voltage stale.';
    else if (limit === null) status = 'Contact threshold unavailable.';
    return {data, limit, name:names[trigger.mux], enabled:trigger.enabled, status, time, gap,
      bounds:adcAxis(null, 'filtered_volts', limit, trigger.mux)};
  };
  function render() {
    if (!latest) return;
    el('apply').disabled = pending || !state.connected || !state.drive.you_control_owner;
    el('status').textContent = `${latest.demo ? 'DEMO · ' : ''}${latest.error || latest.load_error ? 'ERROR' : !latest.running ? 'UNAVAILABLE' : latest.applied !== latest.revision ? 'PENDING' : 'SAMPLING'}`;
    el('error').textContent = latest.error || latest.load_error || (!latest.running ? 'Acquisition is unavailable. Check the ADC connection.' : '');
    renderSensors();
    window.operatorADCFrame?.();
    if (!document.getElementById('settings-dialog').open || document.getElementById('settings-adc').hidden) return;
    const seconds = Number(el('window').value);
    const recent = history.filter(s => now()-s.time <= 2);
    el('rate').textContent = `${(recent.length/2).toFixed(1)} samples/s total · ${latest.config.rate} SPS chip rate shared across enabled inputs`;
    el('readings').innerHTML = latest.config.channels.filter(ch => ch.enabled).map(ch => {
      const samples = history.filter(s => s.mux===ch.mux && now()-s.time<=seconds);
      const last = samples.at(-1), values = samples.map(s=>s.filtered_volts).filter(Number.isFinite);
      const fresh = last && latest.running && latest.connected && now()-last.time < Math.max(1, latest.config.interval_ms/1000 + 2);
      const fmt = n => Number.isFinite(n) ? n.toFixed(6) : '—';
      return `<tr><td>${names[ch.mux]}</td><td>${fresh ? fmt(last.filtered_volts) : '—'}</td><td>${values.length ? [Math.min(...values),Math.max(...values),values.reduce((a,b)=>a+b,0)/values.length].map(fmt).join(' / ') : '—'}</td><td>${!fresh ? 'STALE' : last.clipped ? 'CLIPPED' : 'Live'}</td></tr>`;
    }).join('');
    el('registers').textContent = `${latest.applied === latest.revision ? 'Applied' : 'Applying settings'} · revision ${latest.revision}\n` + Object.entries(latest.registers).map(([k,v])=>`${k}: 0x${v.toString(16).padStart(4,'0')}  ${v.toString(2).padStart(16,'0')}`).join('\n');
  }
  // Plot refresh follows the display; acquisition and table updates do not.
  function previewFilter(points, draftMs) {
    let lastTime = null, filtered = 0;
    return points.map(({x, y}) => {
      if (y === null) { lastTime = null; return {x, y}; }
      const dt = x - lastTime;
      filtered = !draftMs || lastTime === null || dt > .5 ? y : filtered - Math.expm1(-dt * 1000 / draftMs) * (y - filtered);
      lastTime = x;
      return {x, y: filtered};
    });
  }
  window.operatorDetectorFilterPreview = seconds => {
    const draft = smoothingInput.valueAsNumber;
    if (!adcOnline || !smoothingInput.validity.valid || draft === latest.config.filter_ms) return null;
    // Warm the filter using samples preceding the visible window when available.
    const raw = datasets('detector', 'volts', seconds + 10);
    const reference = state.detector.sensor?.reference_voltage;
    if (raw.length !== 1 || !Number.isFinite(reference) || reference <= 0) return null;
    const data = previewFilter(raw[0].data, draft).filter(point => point.x >= -seconds);
    for (const point of data) if (Number.isFinite(point.y)) point.y = Math.max(0, Math.min(reference, point.y));
    return {label:`Preview (${draft} ms)`, data,
      borderColor:getComputedStyle(document.documentElement).getPropertyValue('--warn').trim(),
      borderDash:[7, 3], borderWidth:2, pointRadius:0, spanGaps:false};
  };
  function boardTraces(seconds, compare) {
    const applied = datasets(null, 'filtered_volts', seconds);
    const draftMs = field('filter_ms').valueAsNumber;
    const previewing = field('filter_ms').validity.valid && draftMs !== latest.config.filter_ms;
    if (!compare && !previewing) return applied;
    const unfiltered = datasets(null, 'volts', seconds);
    for (const line of unfiltered) {
      line.borderWidth = 1;
      line.borderDash = [2, 4];
      if (previewing) {
        const data = previewFilter(line.data, draftMs);
        applied.push({...line, label:line.label.replace('unfiltered (V)', 'filter preview (V)'), data, borderWidth:2, borderDash:[7, 3]});
      }
    }
    return compare ? [...unfiltered, ...applied] : applied;
  }
  field('filter_ms').oninput = () => {
    smoothingError = '';
    renderSmoothing();
    if (chart) chart.$frameBucket = null;
    window.operatorADCFrame();
  };
  function renderContactSignal() {
    const mux = Number(muxInput.value);
    const sample = history.findLast(point => point.mux === mux);
    const scanSeconds = latest.config.channels.filter(channel => channel.enabled).length / latest.config.rate;
    const maxAge = Math.max(.5, latest.config.interval_ms / 1000 + 3 * scanSeconds);
    const fresh = adcOnline && latest.connected && latest.running && latest.applied === latest.revision
      && Number.isFinite(sample?.filtered_volts) && now() - sample.time <= maxAge;
    document.getElementById('contact-live-volts').textContent = fresh ? `${sample.filtered_volts.toFixed(4)} V` : '—';
    const draft = document.getElementById('contact-draft-preview');
    draft.hidden = !contactDirty;
    if (!contactDirty) return;
    const level = contactVolts.valueAsNumber;
    const above = contactDirection.value === 'above';
    const inRange = fresh && Number.isFinite(level) && (above ? sample.filtered_volts >= level : sample.filtered_volts <= level);
    draft.textContent = !enabledInput.checked ? 'Draft · detection disabled' : !fresh || !Number.isFinite(level)
      ? 'Draft · waiting for a reading and limit'
      : `Draft · ${names[mux]} · ${sample.filtered_volts.toFixed(4)} V ${inRange ? 'meets' : 'does not meet'} the ${above ? '≥' : '≤'} ${level.toFixed(4)} V limit`;
  }

  function drawContactHistory() {
    if (sourceInput.value === 'esp32') { drawLoadcellHistory(); return; }
    if (contactChart && scrollTimeChart(contactChart, performance.now())) return;
    const mux = Number(muxInput.value);
    const traces = datasets(null, 'filtered_volts', 30, mux);
    const line = traces[0];
    if (line) {
      line.label = `${names[mux]} · filtered V`;
      line.borderColor = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    }
    if (Number.isFinite(contactVolts.valueAsNumber)) {
      const level = contactVolts.valueAsNumber;
      traces.push({label:contactDirty ? 'Draft limit' : enabledInput.checked ? 'Contact limit' : 'Limit (disabled)', data:[{x:-30,y:level},{x:0,y:level}],
        yAxisID:'ads1115', borderColor:getComputedStyle(document.documentElement).getPropertyValue('--warn').trim(),
        borderWidth:1, borderDash:[5,4], pointRadius:0});
    }
    if (!contactChart) contactChart = new Chart(document.getElementById('contact-history'), {
      type:'line', data:{datasets:[]}, options:{animation:false, responsive:true, maintainAspectRatio:false, parsing:false,
        plugins:{decimation:{enabled:true, algorithm:'min-max'}}, scales:{x:{type:'linear', min:-30, max:0,
          title:{display:true,text:'Time (s)'}}, ads1115:{type:'linear',position:'left',title:{display:true,text:'Voltage (V)'}}}}});
    contactChart.options.plugins.decimation.enabled = true;
    contactChart.options.scales.ads1115.title.text = 'Voltage (V)';
    delete contactChart.options.scales.ads1115.min; delete contactChart.options.scales.ads1115.max;
    contactChart.data.datasets = traces;
    contactChart.$scrollPixels = 0;
    contactChart.$renderedAt = performance.now();
    contactChart.update('none');
  }
  function drawLoadcellHistory() {
    if (contactChart && scrollTimeChart(contactChart, performance.now())) return;
    const cell = state.probe?.loadcell, saved = state.settings?.probe_contact;
    const zeroed = cell?.zeroed && Number.isFinite(saved?.zero_counts);
    const elapsed = Math.max(0,(performance.now()-cellHistoryAt)/1000);
    const data = [];
    let hasFiltered = false, hasRaw = false;
    if (state.connected) for (const point of cellHistory) {
      const x = point.x - elapsed;
      const filtered = Number.isFinite(point.filtered_counts);
      const value = filtered ? point.filtered_counts : point.raw_counts;
      if (!Number.isFinite(x) || x < -30 || x > 0 || !Number.isFinite(value)) continue;
      if (filtered) hasFiltered = true; else hasRaw = true;
      if (data.length && x-data.at(-1).x > .5) data.push({x:x-.001,y:null});
      data.push({x,y:zeroed ? Math.abs(value-saved.zero_counts) : value});
    }
    const signal = hasFiltered ? hasRaw ? 'filtered / raw' : 'filtered' : hasRaw ? 'raw' : Number.isFinite(cell?.filtered_counts) ? 'filtered' : 'raw';
    const quantity = zeroed ? `${signal} magnitude from zero (counts)` : `${signal} counts (zero not set)`;
    const color = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    const traces = [{label:`Load cell · ${quantity}`,data,
      yAxisID:'ads1115',borderColor:color('--accent'),borderWidth:1.5,pointRadius:0,spanGaps:false}];
    const level = cellThreshold.valueAsNumber;
    if (zeroed && Number.isFinite(level) && level > 0) traces.push({
      label:`${sourceDirty && sourceChanged() ? 'Draft' : 'Contact'} threshold ${level} counts`,
      data:[{x:-30,y:level},{x:0,y:level}],yAxisID:'ads1115',borderColor:color('--warn'),borderWidth:1,borderDash:[5,4],pointRadius:0});
    if (!contactChart) contactChart = new Chart(document.getElementById('contact-history'), {
      type:'line',data:{datasets:[]},options:{animation:false,responsive:true,maintainAspectRatio:false,parsing:false,
        plugins:{decimation:{enabled:true,algorithm:'min-max'},legend:{labels:{color:color('--muted')}}},scales:{x:{type:'linear',min:-30,max:0,ticks:{color:color('--muted')},title:{display:true,text:'Time (s)',color:color('--muted')}},
        ads1115:{type:'linear',position:'left',title:{display:true,text:'Counts'}}}}});
    const values = traces.flatMap(line => line.data.map(point => point.y)).filter(Number.isFinite);
    const bounds = values.length ? signalBounds(Math.min(...values),Math.max(...values),1) : {};
    contactChart.options.plugins.decimation.enabled = false;
    contactChart.options.plugins.legend.labels.color = color('--muted');
    contactChart.options.scales.x.ticks.color = color('--muted');
    contactChart.options.scales.x.title.color = color('--muted');
    contactChart.options.scales.ads1115 = {type:'linear',position:'left',ticks:{color:color('--muted')},title:{display:true,text:quantity[0].toUpperCase()+quantity.slice(1),color:color('--muted')},...bounds};
    contactChart.data.datasets = traces;
    contactChart.$scrollPixels = 0; contactChart.$renderedAt = performance.now();
    contactChart.update('none');
  }
  function drawBoardHistory() {
    if (chart && scrollTimeChart(chart, performance.now())) return;
    const seconds = Number(el('window').value), quantity = el('quantity').value;
    if (!chart) chart = new Chart(el('history'), {type:'line', data:{datasets:[]}, options:{animation:false, responsive:true, maintainAspectRatio:false, parsing:false,
      plugins:{decimation:{enabled:true,algorithm:'min-max'},legend:{labels:{color:'#b7c4d8'}}}, scales:{x:{type:'linear',min:-seconds,max:0},ads1115:{type:'linear',position:'left'}}}});
    const applied = latest.config.filter_ms;
    const draft = field('filter_ms').valueAsNumber;
    el('filter-status').textContent = Number.isFinite(applied)
      ? `Applied on car: ${applied ? applied+' ms' : 'off'}` + (field('filter_ms').validity.valid && draft !== applied ? ` · Unsaved preview: ${draft ? draft+' ms' : 'off'}` : '')
      : 'Waiting for filter settings from the car';
    const compare = quantity === 'comparison';
    chart.data.datasets = boardTraces(seconds, compare);
    const bounds = adcAxis(null, quantity);
    if (!compare && field('filter_ms').validity.valid && draft !== applied && Number.isFinite(bounds.min)) {
      for (const line of chart.data.datasets) for (const point of line.data) {
        if (!Number.isFinite(point.y)) continue;
        bounds.min = Math.min(bounds.min, point.y);
        bounds.max = Math.max(bounds.max, point.y);
      }
    }
    chart.options.scales.ads1115 = {type:'linear', position:'left', title:{display:true,text:'Voltage (V)'}, ...bounds};
    chart.options.scales.x.min = -seconds; chart.$scrollPixels = 0; chart.$renderedAt = performance.now(); chart.update('none');
  }

  const plots = [
    {panel:'settings-contact', canvas:'contact-history', error:'contact-chart-error', draw:drawContactHistory, reset:() => { contactChart = null; }},
    {panel:'settings-adc', canvas:'adc-history', error:'adc-chart-error', draw:drawBoardHistory, reset:() => { chart = null; }},
  ];
  const probePlot = {canvas:'manual-probe-history', error:'manual-probe-chart-error',
    draw:() => drawManualProbeHistory(performance.now(), true), reset:() => { manualProbeChart = null; }};
  window.operatorADCFrame = () => {
    const setup = document.getElementById('settings-dialog').open;
    const plot = setup ? plots.find(item => !document.getElementById(item.panel).hidden && (latest || (item.panel === 'settings-contact' && sourceInput.value === 'esp32')))
      : !document.querySelector('[data-dock-pane="probe"]').hidden && probePlot;
    if (!plot || performance.now() < (plot.retryAt || 0)) return;
    const message = document.getElementById(plot.error);
    try {
      plot.draw();
      message.hidden = true;
    } catch (error) {
      // Chart.js can register a canvas before its constructor finishes.
      // Release even that partial chart so the next draw can use the canvas.
      plot.retryAt = performance.now() + 1000;
      plot.reset();
      try { Chart.getChart(plot.canvas)?.destroy(); }
      catch (cleanupError) { console.warn('Could not release ADC chart', cleanupError); }
      message.textContent = `Chart unavailable · ${error.message}`;
      message.hidden = false;
    }
  };

  function receive(value) {
    if (session !== value.session) { cursor=0; history=[]; scaleBuckets.clear(); revision=null; session=value.session; }
    if (value.sequence < cursor) { cursor=0; history=[]; scaleBuckets.clear(); revision=null; }
    if (revision !== value.revision) {
      history = [];
      scaleBuckets.clear();
      if (revision === null) fill(value);
      else if (field('filter_ms').valueAsNumber === latest?.config.filter_ms) field('filter_ms').value = value.config.filter_ms;
      revision = value.revision;
    }
    latest=value; adcOnline=true; clockOffset=(value.now-Date.now()/1000);
    rememberScale(value.samples, value.now);
    history.push(...value.samples); cursor=value.samples.at(-1)?.seq ?? cursor;
    history=history.filter(s=>value.now-s.time<=60).slice(-30000);
    if (value.dropped) el('feedback').textContent = `${value.dropped} readings missed during disconnection.`;
    render();
  }

  async function poll() {
    let value;
    try {
      const response = await fetch(`/api/adc?after=${cursor}&session=${encodeURIComponent(session || "")}`, {cache:'no-store', signal:AbortSignal.timeout(3000)});
      if (!response.ok) throw Error(`ADC telemetry: HTTP ${response.status}`);
      value = await response.json();
    } catch (error) {
      adcOnline = false;
      el('status').textContent='DISCONNECTED';
      el('error').textContent=error.message;
      if (pending === 'contact') contactMessage='Contact save was not confirmed. Check the connection.';
      if (pending === 'detector') detectorMessage='Sensor input save was not confirmed. Check the connection.';
      pending=false;
      el('apply').disabled=true;
      renderSensors();
      document.getElementById('probe-adc-contact').textContent='ADC disconnected';
      document.getElementById('probe-map-sensor').textContent='ADC disconnected';
      document.getElementById('contact-live-volts').textContent='—';
      renderContactSource();
    }
    if (value) {
      try { receive(value); }
      catch (error) {
        el('error').textContent = `ADC display error · ${error.message}`;
        el('trigger-state').textContent = el('error').textContent;
      }
    }
    renderContactSource();
    setTimeout(poll, document.hidden ? 1000 : 100);
  }
  poll();
})();
