#ifndef WEB_CONTENT_H
#define WEB_CONTENT_H

// index file
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Float Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 0px auto; padding-top: 30px; background-color: #f2f2f2;}
    .container { max-width: 800px; margin: auto; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}
    .btn { display: inline-block; background-color: #008CBA; border: none; color: white; padding: 16px 40px; text-align: center; text-decoration: none; font-size: 20px; margin: 10px; cursor: pointer; border-radius: 8px; transition: background-color 0.3s;}
    .btn-stop { background-color: #f44336; }
    .btn-manual { background-color: #555555; }
    .btn-apply { background-color: #4CAF50; padding: 10px 30px; font-size: 16px; }
    .btn-download { background-color: #808080; }
    .btn:hover { opacity: 0.8; }
    h1, h2 { color: #333; }
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 20px; text-align: left; }
    .data-item { background-color: #eee; padding: 10px; border-radius: 5px; }
    .data-item span { font-weight: bold; float: right; }
    .target-row { display: flex; justify-content: center; align-items: center; gap: 10px; margin-bottom: 20px; }
    .target-row label { font-weight: bold; }
    .target-row input { width: 100px; padding: 8px; }
    .tuning-grid { display: grid; grid-template-columns: auto 1fr; gap: 10px 20px; align-items: center; margin-top: 15px;}
    .tuning-grid label { text-align: right; font-weight: bold; }
    .tuning-grid input { width: 100%; padding: 8px; box-sizing: border-box; border: 1px solid #ccc; border-radius: 4px; }
    fieldset { border: 1px solid #ccc; border-radius: 8px; padding: 20px; margin-top: 20px; }
    legend { font-weight: bold; font-size: 1.2em; color: #333; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Autonomous Float Control</h1>
    <div>
      <button class="btn" onclick="sendCommand('/start')">Start Mission</button>
      <button class="btn btn-stop" onclick="sendCommand('/stop')">STOP</button>
    </div>
    <div>
      <button class="btn btn-manual" onclick="sendCommand('/top')">Go to Top</button>
      <button class="btn btn-manual" onclick="sendCommand('/bottom')">Go to Bottom</button>
      <button class="btn btn-manual" onclick="sendCommand('/descend')">descending </button>
      <button class="btn btn-manual" onclick="sendCommand('/reset')">reset 0 position </button>
    </div>
    <div class="data-grid">
      <div class="data-item">State: <span id="state">--</span></div>
      <div class="data-item">Depth (m): <span id="depth">--</span></div>
      <div class="data-item">temp (c): <span id="temp">--</span></div>
      <div class="data-item">pressure (bar): <span id="pressure">--</span></div>
      <div class="data-item">ultrasonic_distance (m): <span id="ultrasonic">--</span></div>
      <div class="data-item">Setpoint (m): <span id="setpoint">--</span></div>
      <div class="data-item">Motor Position: <span id="motor_pos">--</span></div>
      <div class="data-item">target step: <span id="target_step">--</span></div>
    </div>
    <fieldset>
      <legend>Tuning Parameters</legend>
      <div class="target-row">
          <label for="target_depth_input">Target Depth (m)(descend to this depth and remain 10 sec then ascending):</label>
          <input type="number" id="target_depth_input" step="0.1" value="0">
          <button class="btn btn-apply" onclick="setTargetDepth()"> Apply Target Depth </button>
      </div>
      <div class="tuning-grid">
        <label for="kp_input">Kp:</label> <input type="number" id="kp_input" step="0.1" value="%KP%">
        <label for="ki_input">Ki:</label> <input type="number" id="ki_input" step="0.1" value="%KI%">
        <label for="kd_input">Kd:</label> <input type="number" id="kd_input" step="0.1" value="%KD%">
        <label for="density_input">Fluid Density:</label><input type="number" id="density_input" step="1" value="%DS%">
        <label for="max_speed">Max Motor Speed:</label> <input type="number" id="max_speed" step="100" value="%MS%">
        <label for="accek">Motor acceleration:</label> <input type="number" id="accel" step="100" value="%AC%">
      </div>
      <button class="btn btn-apply" onclick="applyTuning()">Apply Tuning</button>
    </fieldset>
    <div style="margin-top: 30px;">
      <h2>Mission Data</h2>
      <p>Click the button below to download the data from the last completed mission as a CSV file.</p>
      <a href="/history_csv" download="mission_data.csv"><button class="btn btn-download">Download Mission Data</button></a>
    </div>
  </div>

<script>

  function (url) { 
    fetch(url).then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Error:', error));
  }

  function setTargetDepth()
  {
    const depth = document.getElementById("target_depth_input").value;

    fetch(`/set_depth?depth=${depth}`)
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(error => console.error(error));
  }

  function applyTuning() { 
    const kp = document.getElementById('kp_input').value; 
    const ki = document.getElementById('ki_input').value; 
    const kd = document.getElementById('kd_input').value; 
    const density = document.getElementById('density_input').value;
    const max_speed = document.getElementById('max_speed').value; 
    const accel = document.getElementById('accel').value;  
    const url = `/set_tuning?kp=${kp}&ki=${ki}&kd=${kd}&density=${density}&max_speed=${max_speed}&accel=${accel}`; 
    sendCommand(url); 
  }

  function updateData() { 
    fetch('/data').then(response => response.json()).then(data => { 
      document.getElementById('state').innerText = data.state;
      document.getElementById('depth').innerText = data.depth.toFixed(3);
      document.getElementById('temp').innerText = data.temp.toFixed(3);
      document.getElementById('pressure').innerText = data.pressure.toFixed(3);
      document.getElementById('ultrasonic').innerText = data.ultrasonic.toFixed(3);
      document.getElementById('setpoint').innerText = data.setpoint.toFixed(3);
      document.getElementById('motor_pos').innerText = `${data.motor_pos} / ${data.motor_max}`;
      document.getElementById('target_step').innerText = data.target_step;
    }).catch(error => console.error('Error fetching data:', error)); 
  }

  setInterval(updateData, 1000);
  window.onload = updateData;
</script>
</body>
</html>
)rawliteral";

#endif  // WEB_CONTENT_H