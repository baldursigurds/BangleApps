pinv = function (A,b) {
    m = A.length;
    n = A[0].length;
    // matrix with columns an orthogonal basis of the image of A
    O = [];
    // change of basis matrix from columns of A to columns of O
    P = [];
    // return value, in O coordinates
    ro = [];
    // return value
    r = [];
    for(j=0; j<n; j++) {
        P[j] = [];
        for(k=0; k<n; k++) {
            P[j][k] = 0;
        }
        ro[j] = 0;
        r[j] = 0;
    }
    for(i=0; i<m; i++) {
        O[i] = [];
        for(j=0; j<n; j++) {
            O[i][j] = A[i][j];
        }
    }
    norm = [];
    normsq = [];
    for(j=0; j<n; j++) {
        P[j][j] = 1;
        for(k=0; k<j; k++) {
            for(i=0; i<m; i++) {
                P[k][j] += O[i][k] * A[i][j];
            }
            P[k][j] /= normsq[k];
            for(i=0; i<m; i++) {
                O[i][j] -= P[k][j] * O[i][k];
            }
        }
        normsq[j] = 0;
        for(i=0; i<m; i++) {
            normsq[j] += O[i][j]*O[i][j];
        }
        norm[j] = Math.sqrt(normsq[j]);
    }
    for(j=0; j<n; j++) {
        ro[j] = 0;
        for(i=0; i<m; i++) {
            ro[j] += b[i] * O[i][j];
        }
        ro[j] /= normsq[j];
    }
    for(j=n-1; j>=0; j--) {
        r[j] = ro[j];
        for(k=n-1; k>j; k--) {
            r[j] -= P[j][k]*r[k];
        }
    }
    return(r);
};

function onLine(line,measurements,resolve) {
  //console.log("RECEIVED:"+line);
  var d = line.split(",");
  if (d.length==5 && d[0]=="M") {
    E.showMessage("DATA:\n" + d.slice(1,5).join(',\n'));
    console.log("Got DATA:\n" + d.slice(1,5).join(',\n'));
    ints = [];
    d.slice(1,4).forEach( (s) => {ints.push(parseInt(s));});
    measurements.push(ints);
  }
  if(d.length == 1 && d[0].length == 2 && d[0][0] == "S") {
    console.log("Got S");
    resolve(measurements);
  }
}

function collect_sample_bt(freq,n) {
  return new Promise( (resolve) => {
    CODE =`
    i = 0;
    freq = 20;
    n = 100;
    setWatch(function() {
      Bluetooth.println("S");
      Puck.magOff();
    }, BTN, {edge:"rising", debounce:50});
    Puck.removeAllListeners('mag');
    Puck.on('mag', (m) => {
      if(i%6==0)
        digitalPulse(LED1, 1, 2000/freq)
      if(i%6==2)
        digitalPulse(LED2, 1, 2000/freq)
      if(i%6==4)
        digitalPulse(LED3, 1, 2000/freq)
      if(i>10)
        Bluetooth.println("M," + m.x + "," + m.y + "," + m.z + "," + i);
      if(i<500) {
        i++;
      } else {
        Bluetooth.println("S");
        Puck.magOff();
      }
    });
    Puck.magOn(20);
    `;
    measurements = [];
    NRF.requestDevice({ filters: [{ namePrefix: 'Puck.js' }] })
      .then(function(device) {
      return require("ble_uart").connect(device);
    },function() {
      E.showMessage("Didn't work");
    }).then(function(uart) {
      u = uart;
      var buf = "";
      uart.on('data', function(d) {
        buf += d;
        var l = buf.split("\n");
        buf = l.pop();
        l.forEach((elt) => onLine(elt,measurements,resolve,n));
      });
      uart.write(CODE);
    });
  });
}

function roots_deg3(a,b,c,d) {
  p = (3*a*c - (b*b))/(3*(a*a));
  q = (2*(b*b*b) - 9*a*b*c + 27*(a*a)*d) / (27*(a*a*a));
  r = [];
  for(k=0; k<3; k++) {
    r.push( 2*Math.sqrt(-p/3) * Math.cos( 1/3 * Math.acos( 3*q / (2*p) * Math.sqrt(-3/p)) - 2*Math.PI*k / 3) - b/(3*a));
  }
  return r;
}

function matmult3x3_3x3(A,B) {
  C = [ [ 0,0,0], [0,0,0], [0,0,0] ];
  for(i=0; i<3; i++) {
    for(j=0; j<3; j++) {
      for(k=0; k<3; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

function matmult3x3_3x1(A,B) {
  C = [ 0,0,0];
  for(i=0; i<3; i++) {
    for(k=0; k<3; k++) {
      C[i] += A[i][k] * B[k];
    }
  }
  return C;
}

function mat_transp_3x3(A) {
  return [ [ A[0,0], A[1,0], A[2,0] ],
           [ A[0,1], A[1,1], A[2,1] ],
           [ A[0,2], A[1,2], A[2,2] ] ];
}

function eigenvals_3x3_sym(S) {
  a = 1;
  b = -S[0][0]-S[1][1]-S[2][2];
  c =   S[0][0]*S[1][1] - S[0][1]*S[1][0]
      + S[1][1]*S[2][2] - S[1][2]*S[2][1]
      + S[0][0]*S[2][2] - S[0][2]*S[2][0];
  d = - S[0][0]*S[1][1]*S[2][2]
      - S[0][1]*S[1][2]*S[2][0]
      - S[0][2]*S[1][0]*S[2][1]
      + S[0][0]*S[1][2]*S[2][1]
      + S[0][1]*S[1][0]*S[2][2]
      + S[0][2]*S[1][1]*S[2][0];
  eigenvals = roots_deg3(a,b,c,d);
  eigenvals.sort();
  console.log(eigenvals);
  return eigenvals;
}

function mat_sym_3x3_sqrt(B) {
  l = eigenvals_3x3_sym(B);
  m = [ Math.sqrt(l[0]), Math.sqrt(l[1]), Math.sqrt(l[2]) ];
  a =   m[0] / ( (l[1] - l[0])*(l[2]-l[0]) )
      + m[1] / ( (l[0] - l[1])*(l[2]-l[1]) )
      + m[2] / ( (l[0] - l[2])*(l[1]-l[2]) );
  b =   m[0] * (-l[1]-l[2]) / ( (l[1] - l[0])*(l[2]-l[0]) )
      + m[1] * (-l[0]-l[2]) / ( (l[0] - l[1])*(l[2]-l[1]) )
      + m[2] * (-l[0]-l[1]) / ( (l[0] - l[2])*(l[1]-l[2]) );
  c =   m[0] * (l[1]*l[2]) / ( (l[1] - l[0])*(l[2]-l[0]) )
      + m[1] * (l[0]*l[2]) / ( (l[0] - l[1])*(l[2]-l[1]) )
      + m[2] * (l[0]*l[1]) / ( (l[0] - l[2])*(l[1]-l[2]) );
  A = [ [ c, 0, 0],
        [ 0, c, 0],
        [ 0, 0, c] ];
  for(i=0; i<3; i++) {
    for(j=0; j<3; j++) {
      A[i][j] += b * B[i][j];
      A[i][j] += a * (B[i][0]*B[0][j] + B[i][1]*B[1][j] + B[i][2]*B[2][j]);
    }
  }
  return(A);
}

function fit_to_ellipse(R) {
  m = R.length;
  console.log("R.length = " + R.length);
  mat = [];
  col = [];
  R.forEach( (e) => {
    mat.push([
      e[0]*e[0]-e[2]*e[2],
      e[1]*e[1]-e[2]*e[2],
      2*e[0]*e[1],
      2*e[1]*e[2],
      2*e[0]*e[2],
      -2*e[0],
      -2*e[1],
      -2*e[2],
      1]);
    col.push( -e[2]*e[2] );
  });
  s = pinv(mat,col);
  console.log(s);
  B = [ [ s[0], s[2], s[4]          ],
        [ s[2], s[1], s[3]          ],
        [ s[4], s[3], 1-s[0]-s[1] ] ];
  A = mat_sym_3x3_sqrt(B);
  v = [];
//  for(f=0; f<3; f++) {
//    w = pinv(B, [s[5], s[6], s[7]]);
//    console.log("w = ");
//    console.log(w);
//    v.push( w[0]*s[5] + w[1]*s[6] + w[2]*s[7] )
//  }
  w = pinv(B,[1,0,0]);
  v.push( w[0]*s[5] + w[1]*s[6] + w[2]*s[7] );
  w = pinv(B,[0,1,0]);
  v.push( w[0]*s[5] + w[1]*s[6] + w[2]*s[7] );
  w = pinv(B,[0,0,1]);
  v.push( w[0]*s[5] + w[1]*s[6] + w[2]*s[7] );
  vtBv = 0;
  for(i=0; i<3; i++) {
    for(j=0; j<3; j++) {
      vtBv += v[i] * B[i][j] * v[j];
    }
  }
  R = Math.sqrt(vtBv - s[8]);
  s = {
    transl: v,
    matrix: A,
    radius: R,
  };
  return(s);
}

E.showMessage("Collecting sample data");
collect_sample_bt(20,100)
  .then( (r) => {
  E.showMessage("Sample data obtained");
  console.log("Sample data obtained");
  cal_data = fit_to_ellipse(r);
  l = eigenvals_3x3_sym(cal_data.matrix);
  E.showMessage("Eigenvalues:\n"
                 + l[0].toFixed(4) + ",\n"
                 + l[1].toFixed(4) + ",\n"
                 + l[2].toFixed(4) + "_\n"
                 + "Translation:\n"
                 + cal_data.transl[0].toFixed(4) + ",\n"
                 + cal_data.transl[1].toFixed(4) + ",\n"
                 + cal_data.transl[2].toFixed(4) + "\n");
  if(Bangle.isGPSOn()) {
    filename_json = "calibrtr_" + (Date.now()/1000).toFixed(0) + ".json";
    file_json = require('Storage').open(filename_json,"w");
    gpsf = Bangle.getGPSFix();
  }
  filename_csv  = "calibrtr_" + (Date.now()/1000).toFixed(0) + ".csv";
  file_csv = require('Storage').open(filename_csv,"w");
  r.forEach( (e) => {
    file_csv.write(
      e[0] + ',' +
      e[1] + ',' +
      e[2] + '\n');
  });
});
