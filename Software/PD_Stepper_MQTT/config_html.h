const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>PD Stepper MQTT Setup</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <style>
  body {
    font-family: "Lato", sans-serif;
    background-color: #232324; 
    padding: 15px;
    width: 460px;
    margin:0 auto;
    color: white;
  }
  
  #inputs {
    border-radius: 20px;
    padding: 20px;
    margin: 0 auto;
    background-color: #2e2e2e; 
    width: 400px;
  }
  
  hr {
    width: 350px;
  }
  
  h2 {
    text-align: center;
  }
  
  h1 {
    margin-top: 0px;
    margin-bottom: 15px;
    font-size: 40px;
    text-align: center;
    text-decoration: ;
    color: #fc4903;
    font-family: Tahoma, sans-serif;
    font-weight: normal;
  }
  
  #middle-box {
    width: 320px;
    margin-left: auto;
    margin-right: auto;
    margin-bottom: 20px;
  }
  
  .drop-box {
    float: right;
    height: 25px;
    padding-left: 5px;
    padding-right: 5px;
    color: white;
    background-color: #2b2b2b;
    border-radius: 5px;
    border-style: solid;
    border-color: #919191;
    border-width: 1px;
  }
  
  input[type="text"], input[type="number"], input[type="password"] {
    float: right;
    height: 25px;
    padding-left: 5px;
    padding-right: 5px;
    color: white;
    background-color: #2b2b2b;
    border-radius: 5px;
    border-style: solid;
    border-color: #919191;
    border-width: 1px;
    width: 150px;
  }
  
  #bottom-box {
    text-align: center;
  }
  
  .save-button {
    background-color: #fc4903;
    border: none;
    color: white;
    padding: 14px 30px;
    text-align: center;
    text-decoration: none;
    display: inline-block;
    font-size: 16px;
    margin: 4px 2px;
    transition-duration: 0.4s;
    cursor: pointer;
    border-radius: 10px;
    font-size: 17px;
  }
  
  .container {
    display: inline;
    position: relative;
    padding-left: 35px;
    margin-bottom: 12px;
    margin-left: 160px;
    cursor: pointer;
    font-size: 22px;
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
  }
  
  #enabled1-text {
    padding-top:5px;
    display: inline-block;
    margin-right: 20px;
  }
  
  .container input {
    position: absolute;
    opacity: 0;
    cursor: pointer;
    height: 0;
    width: 0;
  }

  .checkmark {
    position: absolute;
    top: 0;
    left: 0;
    height: 25px;
    width: 25px;
    background-color: #eee;
  }

  .container:hover input ~ .checkmark {
    background-color: #ccc;
  }

  .container input:checked ~ .checkmark {
    background-color: #fc4903;
  }

  .checkmark:after {
    content: "";
    position: absolute;
    display: none;
  }

  .container input:checked ~ .checkmark:after {
    display: block;
  }

  .container .checkmark:after {
    left: 9px;
    top: 5px;
    width: 5px;
    height: 10px;
    border: solid white;
    border-width: 0 3px 3px 0;
    -webkit-transform: rotate(45deg);
    -ms-transform: rotate(45deg);
    transform: rotate(45deg);
  }
  
  label {
    display: block;
    margin-bottom: 10px;
    margin-top: 10px;
  }
  
  </style>
  
</head><body>

  <div id="inputs">

    <h1>PD STEPPER</h1>
    <h2>MQTT Configuration</h2>
  <hr>
  
    <form action="/save" method="post">
      <div id="middle-box">
        <h3>WiFi Settings</h3>
        <label for="wifi_ssid">WiFi SSID:</label>
        <input type="text" id="wifi_ssid" name="wifi_ssid" value="%wifi_ssid%" required>
        <br>
        <label for="wifi_password">WiFi Password:</label>
        <input type="password" id="wifi_password" name="wifi_password" value="">
        <br>
        <br>
        
        <h3>MQTT Settings</h3>
        <label for="mqtt_server">MQTT Server:</label>
        <input type="text" id="mqtt_server" name="mqtt_server" value="%mqtt_server%" required>
        <br>
        <label for="mqtt_port">MQTT Port:</label>
        <input type="number" id="mqtt_port" name="mqtt_port" value="%mqtt_port%" min="1" max="65535">
        <br>
        <label for="mqtt_username">MQTT Username (optional):</label>
        <input type="text" id="mqtt_username" name="mqtt_username" value="%mqtt_username%">
        <br>
        <label for="mqtt_password">MQTT Password (optional):</label>
        <input type="password" id="mqtt_password" name="mqtt_password" value="">
        <br>
        <label for="mqtt_base_topic">MQTT Base Topic:</label>
        <input type="text" id="mqtt_base_topic" name="mqtt_base_topic" value="%mqtt_base_topic%">
        <br>
        <br>
      </div>

      <hr>
      
      <h2>Motor Settings</h2>
      <div id="middle-box">
        <label for="enabled1" id="enabled1-text">Driver Enable:</label>
        <label class="container">
          <input %enabled1% type="checkbox" id="enabled1" name="enabled1" value="Enabled">
          <span class="checkmark"></span>
        </label>
        
        <br>
        <br>
        
        <label for="setvoltage">Voltage:</label>
        <select id="setvoltage" name="setvoltage" class="drop-box">
          <option value="5">5V</option>
          <option value="9">9V</option>
          <option value="12">12V</option>
          <option value="15">15V</option>
          <option value="20">20V</option>
        </select>
        <br>
        <br>

        <label for="microsteps">Microsteps:</label>
        <select id="microsteps" name="microsteps" class="drop-box">
          <option value="1">1</option>
          <option value="4">4</option>
          <option value="8">8</option>
          <option value="16">16</option>
          <option value="32">32</option>
          <option value="64">64</option>
          <option value="128">128</option>
          <option value="256">256</option>
        </select>
        <br>
        <br>

        <label for="current">Max Current:</label>
        <select id="current" name="current" class="drop-box">
          <option value="10">10&#37;</option>
          <option value="20">20&#37;</option>
          <option value="30">30&#37;</option>
          <option value="40">40&#37;</option>
          <option value="50">50&#37;</option>
          <option value="60">60&#37;</option>
          <option value="70">70&#37;</option>
          <option value="80">80&#37;</option>
          <option value="90">90&#37;</option>
          <option value="100">100&#37;</option>
        </select>
        <br>
        <br>

        <label for="stall_threshold">Stall Threshold:</label>
        <select id="stall_threshold" name="stall_threshold" class="drop-box">
          <option value="5">5</option>
          <option value="10">10</option>
          <option value="20">20</option>
          <option value="30">30</option>
          <option value="40">40</option>
          <option value="50">50</option>
          <option value="60">60</option>
          <option value="70">70</option>
          <option value="80">80</option>
          <option value="90">90</option>
          <option value="100">100</option>
          <option value="110">110</option>
          <option value="120">120</option>
          <option value="130">130</option>
          <option value="140">140</option>
          <option value="150">150</option>
        </select>
        <br>
        <br>
        <label for="standstill_mode">Standstill mode:</label>
        <select id="standstill_mode" name="standstill_mode" class="drop-box">
          <option value="NORMAL">Normal</option>
          <option value="FREEWHEELING">Freewheeling</option>
          <option value="BRAKING">Braking</option>
          <option value="STRONG_BRAKING">Strong-Braking</option>
        </select>
        
      </div>

      <div id="bottom-box">
        <input class="save-button" type="submit" value="Save Configuration">
      </div>
    </form>
    
    <br>
    <hr>
    
    <div id="middle-box">
      <h3>MQTT Topics</h3>
      <p><strong>Subscribe to:</strong></p>
      <ul style="text-align: left; font-size: 12px;">
        <li><code>motor/mode</code> - "velocity", "move", or "position"</li>
        <li><code>motor/rpm</code> - Speed in RPM</li>
        <li><code>motor/direction</code> - "cw" or "ccw"</li>
        <li><code>motor/home</code> - true to set home position</li>
        <li><code>motor/conf</code> - JSON config</li>
        <li><code>motor/target</code> - Target distance (e.g., "20" or "20 mm/s")</li>
        <li><code>motor/enable</code> - true/false to enable/disable motor (allows manual rotation when disabled)</li>
      </ul>
      <p><strong>Publishes to:</strong></p>
      <ul style="text-align: left; font-size: 12px;">
        <li><code>motor/position</code> - Current position in mm</li>
      </ul>
    </div>
    
  </div>
  
</body></html>

<script>
  /* Auto fill dropdowns with saved values */ 
  var temp = "%microsteps%";
  var mySelect = document.getElementById('microsteps');
  for(var i, j = 0; i = mySelect.options[j]; j++) {
      if(i.value == temp) {
          mySelect.selectedIndex = j;
          break;
      }
  }
  
  var temp = "%voltage%";
  var mySelect = document.getElementById('setvoltage');
  for(var i, j = 0; i = mySelect.options[j]; j++) {
      if(i.value == temp) {
          mySelect.selectedIndex = j;
          break;
      }
  }
  
  var temp = "%current%";
  var mySelect = document.getElementById('current');
  for(var i, j = 0; i = mySelect.options[j]; j++) {
      if(i.value == temp) {
          mySelect.selectedIndex = j;
          break;
      }
  }
  
  var temp = "%stall_threshold%";
  var mySelect = document.getElementById('stall_threshold');
  for(var i, j = 0; i = mySelect.options[j]; j++) {
      if(i.value == temp) {
          mySelect.selectedIndex = j;
          break;
      }
  }
  
  var temp = "%standstill_mode%";
  var mySelect = document.getElementById('standstill_mode');
  for(var i, j = 0; i = mySelect.options[j]; j++) {
      if(i.value == temp) {
          mySelect.selectedIndex = j;
          break;
      }
  }
</script>

)rawliteral";

