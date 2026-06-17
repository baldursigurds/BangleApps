g.clear();
Bangle.setLocked(0);
Bangle.setLCDTimeout(0);
Bangle.setLCDPower(1);
Bangle.loadWidgets();
Bangle.drawWidgets();


let w = g.getWidth();
let h = g.getHeight();

let startTime;
let time;
let state = "clear";
let runI;
let stopT;

function display() {
  g.clear(1);
  Bangle.drawWidgets();
  g.setFontAlign(0,0);
  g.setFont("14:2");
  g.setColor("#6f4e37");
  let i;
  for(i=4; i<w; i+=9) {
    g.drawLine(i, 24, i, h);
  }
  for(i=4+24; i<h; i+=9) {
    g.drawLine(0, i, w, i);
  }
  g.setColor("#F0E79F");
  if(state == "clear") {
    g.drawString("COFFEE", w/2,(h-24)/3+24);
    g.drawString("TIME" , w/2,(h-24)*2/3+24);
    console.log("We are in clear");
  } else {
    time = (Date.now() - startTime) % 3600000;
    min  = ( time/60000      ).toFixed(0).padStart(2,'0');
    sec  = ((time%60000)/1000).toFixed(0).padStart(2,'0');
    dsec = ((time%1000)/100  ).toFixed(0);
    g.drawString(
      min + ":" + sec + "." + dsec,
      w/2,
      (h+24)/2
    );
  }
}

setInterval(() => {console.log(state);},500);
function onBtn(n,e) {
  if(state == "clear") {
    startTime = Date.now();
    i = setInterval(display,100);
    state = "run";
    Bangle.buzz();
  } else if(state == "run") {
    clearInterval(i);
    display();
    state = "stop";
    Bangle.buzz();
  } else if(state == "stop") {
    state = "stopstop";
    stopT = setTimeout(() => {
      state = "stop";
    }, 500);
  } else if(state == "stopstop") {
    clearTimeout(stopT);
    g.clear();
    state = "clear";
    display();
    Bangle.buzz();
  }
}

display();

Bangle.setUI({
  mode: "custom",
  touch: onBtn
});
