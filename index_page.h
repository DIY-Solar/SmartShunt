const char index_page[] PROGMEM = R"=====(<!DOCTYPE HTML>
<HTML>
  <HEAD>
    <META name='viewport' content='width=device-width, initial-scale=1'>
    <TITLE>SmartShunt live</TITLE>
    <SCRIPT>
    var xmlHttp=createXmlHttpObject();
    var XMLCall;
    XMLCall = false;
    function createXmlHttpObject(){
     if(window.XMLHttpRequest){
        xmlHttp=new XMLHttpRequest();
     }else{
        xmlHttp=new ActiveXObject('Microsoft.XMLHTTP');
     }
     return xmlHttp;
    }
    function process(){
      if(xmlHttp.readyState==0 || xmlHttp.readyState==4){
        xmlHttp.open('PUT','xml',true);
        xmlHttp.onreadystatechange=handleServerResponse;
        xmlHttp.send(null);
      }
      setTimeout('process()',1000);
    }
    function handleServerResponse(){
     if(xmlHttp.readyState==4 && xmlHttp.status==200){
       xmlResponse=xmlHttp.responseXML;
       xmldoc=xmlResponse.getElementsByTagName('A0')[0].firstChild.nodeValue;
       document.getElementById('A0').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('A1')[0].firstChild.nodeValue;
       document.getElementById('A1').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('capacity')[0].firstChild.nodeValue;
       document.getElementById('capacity').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('SoC')[0].firstChild.nodeValue;
       document.getElementById('SoC').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('VBat')[0].firstChild.nodeValue;
       document.getElementById('VBat').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('current')[0].firstChild.nodeValue;
       document.getElementById('current').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('power')[0].firstChild.nodeValue;
       document.getElementById('power').innerHTML=xmldoc;

       if (XMLCall==false) {
         xmldoc=xmlResponse.getElementsByTagName('SoC')[0].firstChild.nodeValue;
         document.getElementById('remain').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('peukert')[0].firstChild.nodeValue;
         document.getElementById('peukert').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('LCSoC')[0].firstChild.nodeValue;
         document.getElementById('LCSoC').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('BLV')[0].firstChild.nodeValue;
         document.getElementById('BLV').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('MCV')[0].firstChild.nodeValue;
         document.getElementById('MCV').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('MCC')[0].firstChild.nodeValue;
         document.getElementById('MCC').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('MDC')[0].firstChild.nodeValue;
         document.getElementById('MDC').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('cap')[0].firstChild.nodeValue;
         document.getElementById('cap').value=xmldoc;

         xmldoc=xmlResponse.getElementsByTagName('TC')[0].firstChild.nodeValue;
         document.getElementById('TC').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('CDT')[0].firstChild.nodeValue;
         document.getElementById('CDT').value=xmldoc;
         xmldoc=xmlResponse.getElementsByTagName('CEF')[0].firstChild.nodeValue;
         document.getElementById('CEF').value=xmldoc;
       }
       XMLCall = true;
       xmldoc=xmlResponse.getElementsByTagName('count')[0].firstChild.nodeValue;
       document.getElementById('count').innerHTML=xmldoc;       
       xmldoc=xmlResponse.getElementsByTagName('ntp')[0].firstChild.nodeValue;
       document.getElementById('ntp').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('upt')[0].firstChild.nodeValue;
       document.getElementById('uptime').innerHTML=xmldoc;
       xmldoc=xmlResponse.getElementsByTagName('freeh')[0].firstChild.nodeValue;
       document.getElementById('freeheap').innerHTML=xmldoc;
     }
    }
    </SCRIPT>
    <STYLE>
      h1 {
        font-size: 120%;
        color: blue;
        margin: 0 0 10px 0;
      }
       table{
        border-collapse: collapse;
      }     
      table, th, td {
        text-align: center;
        border: 1px solid blue;
      }
      tr:nth-child(even) {background-color: #f2f2f2}
    </STYLE>
  </HEAD>
  <BODY onload='process()'>
    <CENTER>
      <H1>SmartShunt live table</H1>
      <TABLE BORDER=1>
        
        <TR><TH>A0</TH><TD><A id='A0'></A></TD><TD>mV</TD></TR>
        <TR><TH>Status</TH><TD><A id='A1'></A></TD><TD>Value</TD></TR>
        <TR><TH>remain cap.</TH><TD><A id='capacity'></A></TD><TD>Ah</TD></TR>
        <TR><TH>SoC</TH><TD><A id='SoC'></A></TD><TD>%</TD></TR>
        <TR><TH>VBat</TH><TD><A id='VBat'></A></TD><TD>V</TD></TR>
        <TR><TH>Current</TH><TD><A id='current'></A></TD><TD>A</TD></TR>
        <TR><TH>Power</TH><TD><A id='power'></A></TD><TD>W</TD></TR>

        <TR><TH title="COUNTER">READ COUNT</TH><TD><A id='count'></A></TD><TD>total</TD></TR>
        <TR><TH title="NTP">NTP</TH><TD><A id='ntp'></A></TD><TD>time</TD></TR>
        <TR><TH title="UPTIME">UPTIME</TH><TD><A id='uptime'></A></TD><TD>d h:m:s</TD></TR>
        <TR><TH title="FREE HEAP">FREE HEAP</TH><TD><A id='freeheap'></A></TD><TD>bytes</TD></TR>
      </TABLE>
      <hr>
      <form action='/' method='post'>
      <table border="1">
      <tbody>
      <tr>
      <td>correct SoC:</td>
      <td><input id="remain" name="remain" step="0.1" type="number" min="0.0" max="100.0"/>%</td>
      </tr>
      <tr>
      <td>Peukert (1,00 - 1,50):</td>
      <td><input id="peukert" name="peukert" step="0.01" type="number" min="1.00" max="1.50"/>k</td>
      </tr>
      <tr>
      <td>Entladeboden:</td>
      <td><input id="LCSoC" name="LCSoC" step="1" type="number" min="0" max="99"/>%</td>
      </tr>
      <tr>
      <td>ALARM BatteryLowVoltage:</td>
      <td><input id="BLV" name="BLV" step="0.01" type="number" min="1.00" max="6553,5"/>V</td>
      </tr>
      <tr>
      <td>MaxChargeVoltage (CVL):</td>
      <td><input id="MCV" name="MCV" step="0.01" type="number" min="1.00" max="60.00"/>V</td>
      </tr>
      <tr>
      <td>mV Shunt:</td>
      <td><input id="MCC" name="MCC" step="0.1" type="number" />mV</td>
      </tr>
      <tr>
      <td>mV Correct:</td>
      <td><input id="MDC" name="MDC" step="0.001" type="number" />mV</td>
      </tr>
      <tr>
      <td>Battery capacity:</td>
      <td><input id="cap" name="cap" step="1" type="number" min="1" max="10000"/>Ah</td>
      </tr>
      <tr>
      <td>Tail current:</td>
      <td><input id="TC" name="TC" step="0.1" type="number" min="0.5" max="10.0"/>%</td>
      </tr>
      <tr>
      <td>Charged detection time:</td>
      <td><input id="CDT" name="CDT" step="1" type="number" min="0" max="100"/>min</td>
      </tr>
      <tr>
      <td>Charge Efficency Factor (CEF):</td>
      <td><input id="CEF" name="CEF" step="1" type="number" min="50" max="100"/>%</td>
      </tr>
      </tbody>
      </table>
      <br>
      <input type="submit" value="Update" />
      </form>
      <br>
      <a href='/reset'>RESET History Data</a>
      <hr>
      <a href='/status'>JSON Status Page</a>
      <br>
      <a href='/update'>FIRMWARE Update</a>
    </CENTER>
  </BODY>
</HTML>)=====";
