let websocket_client;
        let isConnected = false;
        let chartWindowSec = 30;

        function updateChartWindow() {
            chartWindowSec = parseInt(document.getElementById('chartWindowSec').value) || 30;

            // Rebuild the chart instantly from export buffer
            velChart.data.labels = []; velChart.data.datasets[0].data = [];
            accChart.data.labels = []; accChart.data.datasets[0].data = [];
            curChart.data.labels = []; curChart.data.datasets[0].data = [];
            z1Chart.data.labels = []; z1Chart.data.datasets[0].data = [];
            z2Chart.data.labels = []; z2Chart.data.datasets[0].data = [];
            z3Chart.data.labels = []; z3Chart.data.datasets[0].data = [];

            if (exportDataBuffer && exportDataBuffer.length > 0) {
                const latestTime = exportDataBuffer[exportDataBuffer.length - 1].time;
                const targetTime = latestTime - chartWindowSec;

                let startIndex = 0;
                for (let i = exportDataBuffer.length - 1; i >= 0; i--) {
                    if (exportDataBuffer[i].time < targetTime) {
                        startIndex = i + 1;
                        break;
                    }
                }

                for (let i = startIndex; i < exportDataBuffer.length; i++) {
                    if (i % 5 !== 0) continue; // match visual decimation
                    const telemetry_point = exportDataBuffer[i];
                    const label = telemetry_point.time.toFixed(2);
                    velChart.data.labels.push(label); velChart.data.datasets[0].data.push(telemetry_point.velocity);
                    accChart.data.labels.push(label); accChart.data.datasets[0].data.push(telemetry_point.acceleration);
                    curChart.data.labels.push(label); curChart.data.datasets[0].data.push(telemetry_point.current);
                    z1Chart.data.labels.push(label); z1Chart.data.datasets[0].data.push(telemetry_point.z1);
                    z2Chart.data.labels.push(label); z2Chart.data.datasets[0].data.push(telemetry_point.z2);
                    z3Chart.data.labels.push(label); z3Chart.data.datasets[0].data.push(telemetry_point.z3);
                }
            }

            velChart.update('none'); accChart.update('none'); curChart.update('none');
            z1Chart.update('none'); z2Chart.update('none'); z3Chart.update('none');
        }

        // Exact conversions from original C++ controlwidget.cpp
        const CONTROL_PERIOD = 0.001;
        const CONTROL_FREQ = 1000.0;
        const OUTPUT_SCALE = 1.0;

        // ตัวแปรเก็บ Baseline สำหรับกระบวนการ Cross-fade
        let currentActiveParams = {
            position: { c0: 0.5, c1: 0.0, c2: 0.1, d1: 1.0 },
            velocity: { c0: 0.1, c1: 0.01, c2: 0.0, d1: 1.0 },
            current: { c0: 1.0, c1: 0.1, c2: 0.0, d1: 1.0 }
        };


        function syncInput(sourceId, targetId) { document.getElementById(targetId).value = document.getElementById(sourceId).value; }

        async function switchTab(tabPrefix, opModeInt) {
            document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
            document.querySelectorAll('.tab-buttons button').forEach(el => el.classList.remove('active'));
            document.getElementById('tab-' + tabPrefix).classList.add('active');
            document.getElementById('btn-' + tabPrefix).classList.add('active');
            await fetch('/set_op_mode', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode: opModeInt }) });
        }

        // C++ controlwidget.cpp label mapping
        function updateLabels(prefix) {
            const type = document.getElementById(prefix + 'CtrlType').value;
            if (type === 'leadlag') {
                document.getElementById(prefix + 'L1').innerText = 'K';
                document.getElementById(prefix + 'L2').innerText = 'a';
                document.getElementById(prefix + 'L3').innerText = 'b';
            } else if (type === 't_pid') {
                document.getElementById(prefix + 'L1').innerText = 'Kc';
                document.getElementById(prefix + 'L2').innerText = 'Ti';
                document.getElementById(prefix + 'L3').innerText = 'Td';
            } else {
                document.getElementById(prefix + 'L1').innerText = 'P';
                document.getElementById(prefix + 'L2').innerText = 'I';
                document.getElementById(prefix + 'L3').innerText = 'D';
            }
        }

        // Adapted from ControlWidget::on_gain_*_editingFinished() C++ logic
        async function setPID(mode, prefix) {
            const type = document.getElementById(prefix + 'CtrlType').value;
            const val1 = parseFloat(document.getElementById(prefix + '1').value) || 0;
            const val2 = parseFloat(document.getElementById(prefix + '2').value) || 0;
            const val3 = parseFloat(document.getElementById(prefix + '3').value) || 0;
            const limit_i = parseInt(document.getElementById(prefix + 'LimI').value) || 30000;
            const blend_el = document.getElementById('sliderBlend');
            const blend_pct = blend_el ? (parseInt(blend_el.value) || 0) : 0;


            let payload = { mode: mode, p: 0, i: 0, d: 0, gain_output: 1.0, limit_i: limit_i, blend: blend_pct };

            if (type === 'leadlag') {
                payload.p = val1;
                payload.i = (val2 - val3) * val1 * CONTROL_PERIOD;
                payload.d = 0.0;
                payload.gain_output = val3 * CONTROL_PERIOD;
            } else if (type === 'c_pid') {
                payload.p = val1;
                payload.i = val2 * CONTROL_PERIOD;
                payload.d = val3 * CONTROL_FREQ;
            } else if (type === 't_pid') {
                payload.p = val1;
                // Protect against divide by zero
                payload.i = val2 !== 0 ? (val1 * CONTROL_PERIOD / val2) : 0;
                payload.d = val1 * val3 * CONTROL_FREQ;
            } else {
                // Discrete PID (D_PID default)
                payload.p = val1;
                payload.i = val2;
                payload.d = val3;
            }

            currentActiveParams[mode] = { c0: payload.p, c1: payload.i, c2: payload.d, d1: 1.0 };

            await fetch('/set_pid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        }

        async function setADRC(mode, prefix) {
            const wc = parseFloat(document.getElementById(prefix + '_adrc_wc').value) || 0;
            const b0 = parseFloat(document.getElementById(prefix + '_adrc_b0').value) || 0;
            const ramp_time = parseFloat(document.getElementById(prefix + '_adrc_ramp').value) || 0;
            const payload = { mode: mode, wc: wc, b0: b0, ramp_time: ramp_time };
            await fetch('/set_adrc', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        }

        // CSV Export functionality replacing BatchProcessor
        function exportCSV() {
            if (exportDataBuffer.length === 0) {
                alert("No data to export!");
                return;
            }
            let csvContent = "data:text/csv;charset=utf-8,UnixEpoch,Time(s),Velocity(RPM),Acceleration(RPM/s),Current(mA),z1,z2,z3,FadingValue(%),ADRC_Wc,ADRC_b0,ADRC_Ramp\n";
            for (let i = 0; i < exportDataBuffer.length; i++) {
                const telemetry_point = exportDataBuffer[i];
                csvContent += `${telemetry_point.unix_time || ''},${telemetry_point.time},${telemetry_point.velocity},${telemetry_point.acceleration},${telemetry_point.current},${telemetry_point.z1},${telemetry_point.z2},${telemetry_point.z3},${telemetry_point.fading || 0},${telemetry_point.adrc_wc || 0},${telemetry_point.adrc_b0 || 0},${telemetry_point.adrc_ramp || 0}\n`;
            }

            const encodedUri = encodeURI(csvContent);
            const link = document.createElement("a");
            link.setAttribute("href", encodedUri);
            link.setAttribute("download", `telemetry_export_${new Date().getTime()}.csv`);
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
        }

        async function resetPosition() {
            await fetch('/reset_position', { method: 'POST' });
        }

        async function resetAdrc() {
            await fetch('/reset_adrc', { method: 'POST' });
        }

        async function setSysID() {
            const type = parseInt(document.getElementById('sysidType').value);
            const amp = parseInt(document.getElementById('numSysIdAmp').value);
            const freqHz = parseFloat(document.getElementById('numSysIdFreq').value);
            const freq = Math.round(freqHz * 100);
            const offset = parseInt(document.getElementById('numSysIdOffset').value);
            const isSine = (type === 2);

            let payload = { waveform_type: type, amplitude: amp, frequency: freq, offset: offset, sine_enable: isSine };
            await fetch('/set_sysid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        }

        Chart.defaults.color = '#64748b';
        Chart.defaults.borderColor = 'rgba(0,0,0,0.05)';
        const createChart = (ctxId, label, color) => {
            return new Chart(document.getElementById(ctxId), {
                type: 'line',
                data: { labels: [], datasets: [{ label: label, borderColor: color, data: [], fill: false, pointRadius: 0, borderWidth: 2, tension: 0.1 }] },
                options: { animation: false, responsive: true, maintainAspectRatio: false, scales: { x: { display: true }, y: { display: true } } }
            });
        };

        const velChart = createChart('velChart', 'Velocity (rpm)', '#0284c7');
        const accChart = createChart('accChart', 'Acceleration (rpm/s)', '#d97706');
        const curChart = createChart('curChart', 'Current (mA)', '#e11d48');
        const z1Chart = createChart('z1Chart', 'z1 (Position)', '#10b981');
        const z2Chart = createChart('z2Chart', 'z2 (Velocity)', '#8b5cf6');
        const z3Chart = createChart('z3Chart', 'z3 (Accel Disturbance)', '#ec4899');

        async function refreshPorts() {
            const res = await fetch('/ports'); const data = await res.json();
            const select = document.getElementById('portSelect'); select.innerHTML = '';
            data.ports.forEach(p => select.add(new Option(p, p)));
        }

        async function connectPort() {
            const port = document.getElementById('portSelect').value;
            const statusSpan = document.getElementById('connStatus');
            const connBtn = document.getElementById('connBtn');
            statusSpan.innerText = "Connecting..."; connBtn.disabled = true;
            const res = await fetch('/connect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ port: port, device_id: 48 }) });
            const data = await res.json();
            statusSpan.innerText = data.message; statusSpan.style.color = data.status === 'connected' ? "green" : "red";
            if (data.status === 'connected') { isConnected = true; connBtn.innerText = "Disconnect"; connBtn.classList.add('danger-btn'); startTelemetry(); }
            connBtn.disabled = false;
        }

        async function disconnectPort() {
            const res = await fetch('/disconnect', { method: 'POST' }); const data = await res.json();
            document.getElementById('connStatus').innerText = data.message; document.getElementById('connStatus').style.color = "gray";
            if (websocket_client) { websocket_client.close(); websocket_client = null; }
            isConnected = false; const connBtn = document.getElementById('connBtn'); connBtn.innerText = "Connect"; connBtn.classList.remove('danger-btn');
        }

        async function toggleConnect() { if (isConnected) { await disconnectPort(); } else { await connectPort(); } }

        async function setTarget(mode, inputId, minLim, maxLim) {
            let val = parseInt(document.getElementById(inputId).value, 10) || 0;
            const payload = { mode: mode, value: val, min_limit: minLim, max_limit: maxLim };
            await fetch('/set_target', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        }

        async function setPWM() {
            let val = parseInt(document.getElementById('valPwm').value, 10) || 0;
            const payload = { value: val };
            await fetch('/set_pwm', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        }

        async function startMotor() { await fetch('/start', { method: 'POST' }); }
        async function coastStop() { await fetch('/stop', { method: 'POST' }); }

        let incomingBuffer = [];
        let exportDataBuffer = [];
        const maxExportDataPoints = 60000; // 60 seconds at 1000Hz
        let chartDirty = false;

        // Downsampling counter for visual charts to reduce lag
        let renderDecimationCounter = 0;

        // Acceleration derivation state
        let lastTime = null;
        let lastVel = 0;
        let smoothedAcc = 0;
        const accAlpha = 0.05; // Strong low-pass filter to smooth numeric derivative

        function startTelemetry() {
            if (websocket_client) websocket_client.close();
            websocket_client = new WebSocket(`ws://${location.host}/ws/telemetry`);
            websocket_client.onmessage = (event) => {
                const telemetry_points = JSON.parse(event.data);
                for (let i = 0; i < telemetry_points.length; i++) {
                    if (telemetry_points[i].type === 'transfer_progress') {
                        document.getElementById('fadeProgress').value = telemetry_points[i].progress;
                        continue;
                    }
                        // Calculate discrete acceleration dv/dt
                        let telemetry_point = telemetry_points[i];
                        if (lastTime !== null) {
                            const dt = telemetry_point.time - lastTime;
                            if (dt > 0.0001) {
                                const rawAcc = (telemetry_point.velocity - lastVel) / dt;
                                smoothedAcc = (accAlpha * rawAcc) + ((1 - accAlpha) * smoothedAcc);
                            }
                        }
                        telemetry_point.acceleration = smoothedAcc;
                        lastTime = telemetry_point.time;
                        lastVel = telemetry_point.velocity;

                        // Append current UI state
                        const sliderBlend = document.getElementById('sliderBlend');
                        telemetry_point.fading = sliderBlend ? sliderBlend.value : 0;

                        const wc = document.getElementById('vel_adrc_wc');
                        const b0 = document.getElementById('vel_adrc_b0');
                        const ramp = document.getElementById('vel_adrc_ramp');
                        telemetry_point.adrc_wc = wc ? wc.value : 0;
                        telemetry_point.adrc_b0 = b0 ? b0.value : 0;
                        telemetry_point.adrc_ramp = ramp ? ramp.value : 0;

                        // Push to export buffer immediately (full resolution)
                        exportDataBuffer.push(telemetry_point);

                        // Auto clear data log: keep only the last 60 seconds of data
                        while (exportDataBuffer.length > 0 && (telemetry_point.time - exportDataBuffer[0].time > 60.0)) {
                            exportDataBuffer.shift();
                        }

                        // Push to incoming buffer for rendering
                        incomingBuffer.push(telemetry_point);
                }
                chartDirty = true;
            };
        }

        function renderLoop() {
            if (chartDirty && incomingBuffer.length > 0) {
                chartDirty = false;
                const telemetry_points = incomingBuffer; incomingBuffer = [];
                // Only plot every 10th point (100Hz visual refresh) to save rendering performance
                const visualDecimation = 10;

                for (let i = 0; i < telemetry_points.length; i++) {
                    renderDecimationCounter++;
                    if (renderDecimationCounter % visualDecimation !== 0) continue;

                    const telemetry_point = telemetry_points[i]; const label = telemetry_point.time.toFixed(2);
                    velChart.data.labels.push(label); velChart.data.datasets[0].data.push(telemetry_point.velocity);
                    accChart.data.labels.push(label); accChart.data.datasets[0].data.push(telemetry_point.acceleration);
                    curChart.data.labels.push(label); curChart.data.datasets[0].data.push(telemetry_point.current);

                    z1Chart.data.labels.push(label); z1Chart.data.datasets[0].data.push(telemetry_point.z1);
                    z2Chart.data.labels.push(label); z2Chart.data.datasets[0].data.push(telemetry_point.z2);
                    z3Chart.data.labels.push(label); z3Chart.data.datasets[0].data.push(telemetry_point.z3);
                }

                // Batch remove excess points based on real time
                if (velChart.data.labels.length > 0) {
                    const latestTime = parseFloat(velChart.data.labels[velChart.data.labels.length - 1]);
                    const threshold = latestTime - chartWindowSec;

                    let excess = 0;
                    for (let i = 0; i < velChart.data.labels.length; i++) {
                        if (parseFloat(velChart.data.labels[i]) < threshold) {
                            excess++;
                        } else {
                            break;
                        }
                    }

                    if (excess > 0) {
                        velChart.data.labels.splice(0, excess); velChart.data.datasets[0].data.splice(0, excess);
                        accChart.data.labels.splice(0, excess); accChart.data.datasets[0].data.splice(0, excess);
                        curChart.data.labels.splice(0, excess); curChart.data.datasets[0].data.splice(0, excess);

                        z1Chart.data.labels.splice(0, excess); z1Chart.data.datasets[0].data.splice(0, excess);
                        z2Chart.data.labels.splice(0, excess); z2Chart.data.datasets[0].data.splice(0, excess);
                        z3Chart.data.labels.splice(0, excess); z3Chart.data.datasets[0].data.splice(0, excess);
                    }
                }

                velChart.update('none'); accChart.update('none'); curChart.update('none');
                z1Chart.update('none'); z2Chart.update('none'); z3Chart.update('none');
            }
            requestAnimationFrame(renderLoop);
        }
        requestAnimationFrame(renderLoop);

        refreshPorts(); switchTab('vel', -2);

        let aiChatDragDist = 0; let isDraggingAiChat = false; let aiChatStartX = 0; let aiChatStartLeft = 0;
        document.addEventListener('DOMContentLoaded', () => {
            const aiHeader = document.querySelector('.ai-chat-header'); const aiPanel = document.getElementById('aiChatPanel');
            aiHeader.addEventListener('mousedown', (e) => { if (e.target.id === 'aiChatToggleIcon') return; isDraggingAiChat = true; aiChatDragDist = 0; aiChatStartX = e.clientX; aiChatStartLeft = parseInt(window.getComputedStyle(aiPanel).left, 10) || 20; aiHeader.style.cursor = 'grabbing'; });
            document.addEventListener('mousemove', (e) => { if (!isDraggingAiChat) return; const dx = e.clientX - aiChatStartX; aiChatDragDist += Math.abs(e.movementX); let newLeft = aiChatStartLeft + dx; const maxLeft = window.innerWidth - aiPanel.offsetWidth; if (newLeft < 0) newLeft = 0; if (newLeft > maxLeft) newLeft = maxLeft; aiPanel.style.left = newLeft + 'px'; aiPanel.style.right = 'auto'; });
            document.addEventListener('mouseup', () => { if (isDraggingAiChat) { isDraggingAiChat = false; aiHeader.style.cursor = 'pointer'; } });
        });

        function toggleChat() { if (aiChatDragDist > 3) return; const panel = document.getElementById('aiChatPanel'); const icon = document.getElementById('aiChatToggleIcon'); panel.classList.toggle('closed'); icon.innerText = panel.classList.contains('closed') ? '▲' : '▼'; }
        function handleChatKeyPress(e) { if (e.key === 'Enter') { sendChatMessage(); } }

        async function sendChatMessage() {
            const input = document.getElementById('aiChatInput'); const chat_message = input.value.trim(); if (!chat_message) return; input.value = '';
            const body = document.getElementById('aiChatBody');
            const userDiv = document.createElement('div'); userDiv.className = 'chat-message user-message'; userDiv.innerText = chat_message; body.appendChild(userDiv); body.scrollTop = body.scrollHeight;

            let activeTab = 'Unknown';
            if (document.getElementById('tab-pos').classList.contains('active')) activeTab = 'Position';
            if (document.getElementById('tab-vel').classList.contains('active')) activeTab = 'Velocity';

            const context = { activeTab: activeTab, target: document.getElementById('numPos').value };

            const loadingDiv = document.createElement('div'); loadingDiv.className = 'chat-message ai-message'; loadingDiv.innerText = 'Thinking...'; body.appendChild(loadingDiv); body.scrollTop = body.scrollHeight;
            try { const res = await fetch('/chat', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ message: chat_message, context: context }) }); const data = await res.json(); let cleanText = data.response.replace(/\*\*/g, ''); loadingDiv.innerText = cleanText; }
            catch (e) { loadingDiv.innerText = "Error connecting to AI Tutor."; }
            body.scrollTop = body.scrollHeight;
        }

        async function startSafeTransfer(mode, prefix) {
            const type = document.getElementById(prefix + 'CtrlType').value;
            const val1 = parseFloat(document.getElementById(prefix + '1').value) || 0;
            const val2 = parseFloat(document.getElementById(prefix + '2').value) || 0;
            const val3 = parseFloat(document.getElementById(prefix + '3').value) || 0;
            const limit_i = parseInt(document.getElementById(prefix + 'LimI').value) || 30000;

            // คำนวณค่าเป้าหมายของ Lead-Lag หรือ PID ตัวใหม่ ที่ w = 1.0
            let c_new0 = val1;
            let c_new1 = 0, c_new2 = 0, gain_B_new = 0;

            if (type === 'leadlag') {
                c_new1 = (val2 - val3) * val1 * CONTROL_PERIOD;
                gain_B_new = val3 * CONTROL_PERIOD;
            } else {
                // กรณี Target เป็นแบบอื่นๆ (อิงจากสมการใน setPID เดิม)
                c_new1 = val2;
                c_new2 = val3;
            }

            // แปลงกลับจาก gain_B เป็น d_new1 เพื่อส่งให้ Backend ตามสมการ d_1(w)
            let d_new1 = 1.0 / (gain_B_new + 1.0);

            const payload = {
                mode: mode,
                c_pid0: currentActiveParams[mode].c0,
                c_pid1: currentActiveParams[mode].c1,
                c_pid2: currentActiveParams[mode].c2,
                c_new0: c_new0,
                c_new1: c_new1,
                c_new2: c_new2,
                d_new1: d_new1,
                limit_i: limit_i
            };

            document.getElementById('fadeProgress').value = 0;

            try {
                const res = await fetch('/safe_transfer', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });
                const data = await res.json();

                if (data.status === "aborted") {
                    alert("⚠️ " + data.reason);
                    document.getElementById('fadeProgress').value = 0;
                } else {
                    // อัปเดต Baseline ใหม่เมื่อทำ Transfer สำเร็จ
                    currentActiveParams[mode] = { c0: c_new0, c1: c_new1, c2: c_new2, d1: d_new1 };
                    document.getElementById('fadeProgress').value = 100;
                }
            } catch (e) {
                console.error("Transfer Error:", e);
            }
        }
        function updateBlendUI() {
            // Replaced by syncInput directly in HTML
        }

        async function sendBlendData() {
            // ใช้ฟังก์ชัน setPID เดิมแต่ดึงค่า blend ส่งแถมไปด้วย
            // หากอยู่แท็บ Velocity ก็ให้เรียกของ Velocity
            if (document.getElementById('tab-vel').classList.contains('active')) {
                setPID('velocity', 'vel');
            } else if (document.getElementById('tab-pos').classList.contains('active')) {
                setPID('position', 'pos');
            } else {
                setPID('current', 'cur');
            }
        }

        // ⚠️ ให้คุณไปแก้บรรทัดสร้าง payload ภายในฟังก์ชัน setPID() เดิมของคุณ ให้พ่วงค่า blend ไปด้วย
        // โดยเพิ่มบรรทัดนี้เข้าไปในฟังก์ชัน setPID:
        // let blend_pct = parseInt(document.getElementById('sliderBlend').value) || 0;
        // แล้วเสริมเข้าไปใน payload: { mode: mode, p: ... , blend: blend_pct };

        const resizer = document.getElementById('dragResizer'); const leftPanel = document.getElementById('controlsLeft'); let isResizing = false;
        resizer.addEventListener('mousedown', function (e) { e.preventDefault(); isResizing = true; document.body.style.cursor = 'col-resize'; });
        document.addEventListener('mousemove', function (e) { if (!isResizing) return; const newWidth = e.clientX - leftPanel.getBoundingClientRect().left; if (newWidth > 250 && newWidth < window.innerWidth - 200) { leftPanel.style.width = newWidth + 'px'; leftPanel.style.maxWidth = 'none'; } });
        document.addEventListener('mouseup', function (e) { if (isResizing) { isResizing = false; document.body.style.cursor = 'default'; } });

        const graphResizer = document.getElementById('graphDragResizer');
        const graphLeftPanel = document.getElementById('graphsLeftColumn');
        let isGraphResizing = false;

        graphResizer.addEventListener('mousedown', function (e) {
            e.preventDefault();
            isGraphResizing = true;
            document.body.style.cursor = 'col-resize';
        });

        document.addEventListener('mousemove', function (e) {
            if (!isGraphResizing) return;
            const graphsWrapper = document.getElementById('graphsWrapper');
            const newWidth = e.clientX - graphsWrapper.getBoundingClientRect().left;
            if (newWidth > 150 && newWidth < graphsWrapper.clientWidth - 150) {
                graphLeftPanel.style.flex = 'none';
                graphLeftPanel.style.width = newWidth + 'px';
            }
        });

        document.addEventListener('mouseup', function (e) {
            if (isGraphResizing) {
                isGraphResizing = false;
                document.body.style.cursor = 'default';
            }
        });