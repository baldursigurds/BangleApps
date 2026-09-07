// the accrnr module
var arm = require("armod");
const bufferSize = arm.bufferSize;
var bufferIdx = 0;

// connection constants
const PMD_SERVICE = "fb005c80-02e7-f387-1cad-8acd2d8df0c8";
const PMD_CTRL = "fb005c81-02e7-f387-1cad-8acd2d8df0c8";
const PMD_DATA = "fb005c82-02e7-f387-1cad-8acd2d8df0c8";
const HR_SERVICE = "180D";
const HR_CHAR = "2A37";
const POLAR_NAME = "Polar H10";
const MSG = new Uint8Array([
  0x02, 0x02,
  0x02,        // setting_type(RANGE)
  0x01,        // array_length(1)
  0x08, 0x00,  // 8G
  0x00,        // setting_type(SAMPLE_RATE)
  0x01,        // array_length(1)
  0xC8, 0x00,  // 200hz
  0x01,        // setting_type(RESOLUTION)
  0x01,        // array_length(1)
  0x10, 0x00   // 16bit
]);

// connection variables
var gatt;
var pmdDataChar;
var pmdCtrlChar;

// file variables
const dataFileName = "accrnrdta.bin";
const dataFileSize = 7*1024*1024;
var dataFileFull = false;
var recordData = true;
var fileIdx = 0;
var s;
if(recordData) {
  s = require("Storage");
  if(!s.write(dataFileName, "", 0, dataFileSize)) {
    recordData = false;
    console.log("Failed to initialize data file");
  }
}

// display variables
const w = g.getWidth();
const h = g.getHeight();
var blinker = true;

// main global variables
var idx = 0;
var speed = 0;
var hrm = 0;

// buffer variables
var B = new Float32Array(3*256);
var B_addr = E.getAddressOf(B,true);

function handleAccData(e) {
  // data
  let D = e.target.value.buffer;
  let D_addr = E.getAddressOf(D, true) + 10;
  let bl = D.byteLength;
  let l = (bl - 10)/6;

  // write data
  if(recordData && !dataFileFull) {
    if(fileIdx+bl > dataFileSize) {
      dataFileFull = true;
    } else {
      s.write(dataFileName, D, fileIdx, dataFileSize);
      fileIdx += bl;
    }
  }

  // process data
  while(l) {
    let a = bufferIdx;
    if(a+l > bufferSize) {
      b = bufferSize;
      l -= bufferSize - a;
    } else {
      b = a+l;
      l = 0;
    }
    // scale data and add to buffer
    arm.float_win(a, b, D_addr, B_addr);
    bufferIdx = (bufferIdx + b-a)%bufferSize;
    if(bufferIdx == 0) {
      let time = Date.now();
      speed = arm.predict(B);
      time = Date.now() - time;
      console.log("artificial neural network time: ", time);
    }
  }
}

function display() {
  g.clear();
  g.setColor("#000000");
  g.fillRect(0,24,w,h);
  Bangle.drawWidgets();
  g.setFontAlign(0,0);
  g.setFont("14:3");
  g.setColor("#ffffff");
  //if(blinker)
  //  g.drawString("blinker", w/2, (h-24)/4 + 24);
  blinker = !blinker;
  if(recordData)
    if(!dataFileFull)
      g.drawString("FILE: " + (100*fileIdx/dataFileSize).toFixed(0) + "%", w/2, (h-24)/4 + 24); 
    else
      g.drawString("FILE: 100%" , w/2, (h-24)/4 + 24); 
  else
    g.drawString("NO FILE" , w/2, (h-24)/4 + 24); 
  g.drawString("HRM: " + hrm, w/2, (h-24)/2 + 24);
  g.drawString("SPD: " + (speed*3.6).toFixed(1), w/2, (h-24)*3/4 + 24);
}

function log(s) {
  console.log(s);
  //display();
}

function connect() {
  NRF.requestDevice({
      filters: [{
        active: true,
        services: [HR_SERVICE]
      }]
  }).then((device) => {
    console.log("Found device: " + device);
    return device.gatt.connect();
  }).then((g) => {
    gatt = g;
    log("Connected");

    // set up heart rate
    return gatt.getPrimaryService(HR_SERVICE).then((hrService) => {
      return hrService.getCharacteristic(HR_CHAR);
    }).then((hrChar) => {
      hrChar.on('characteristicvaluechanged', (ev) => {
        hrm = ev.target.value.getUint8(1);
	//display();
      });
      return hrChar.startNotifications();
    }).then(() => {
      log("Heart rate ready");
      return gatt.getPrimaryService(PMD_SERVICE);
    });
  
  }).then(pmdService => {
    log("Getting PMD data characteristic");
    return pmdService.getCharacteristic(PMD_DATA);
  }).then(char1 => {
    pmdDataChar = char1;
    log("Getting PMD control characteristic");
    return pmdDataChar.service.getCharacteristic(PMD_CTRL);
  }).then(char2 => {
    pmdCtrlChar = char2;

    pmdDataChar.on('characteristicvaluechanged', handleAccData);
  
    return pmdDataChar.startNotifications();
  }).then(() => {
    log("Sending control command");
    return pmdCtrlChar.writeValue(MSG);
  }).then(() => {
    log("Listening for acc data");
  }).catch((e) => console.log("Error: ", e));
}

Bangle.loadWidgets();
setInterval(display, 1000);
setTimeout(connect, 1000);
