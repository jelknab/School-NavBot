const fileUrl = 'http://localhost:5000/' // provide file location
let tValue;

function getData(){
  fetch(fileUrl)
      .then( r => r.text() )
      .then( t => {
        console.log(typeof t)
        console.log(t)

        monthArray2d = t.split(";").map(function(e) {
          return e.split(",");
        })
        console.log(monthArray2d)
        // outputs the content of the text file
        var data = [
          {
              z: monthArray2d,
              // reversescale: true,
              // x: ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday'],
              // y: ['Morning', 'Afternoon', 'Evening'],
              type: 'heatmap',
              colorscale:[
                  ['0.0', 'rgb(255,255,255)'],
                  ['0.9999', 'rgb(46, 182, 44)'],
                  ['1.0', 'rgb(255,0,0)' ]
              ],
              xgap :	1,
              ygap :	1,
              hoverongaps: false

          }
        ];

        var layout = {
              autosize: false,
              height: 900,
              width: 1400,
              yaxis: {
                  dtick: 1,
                  showgrid: false,
                  zeroline: false
              },
              xaxis: {
                  dtick: 1,
                  showgrid: false,
                  zeroline: false
              },
              automargin: true,
              // margin: {
              //     l: 20,
              //     r: 20,
              //     b: 50,
              //     t: 50,
              //     pad: 2
              // },
              // paper_bgcolor: '#7f7f7f',
              plot_bgcolor: '#c7c7c7'
          };

        Plotly.newPlot('myDiv', data, layout);
      } )
}

getData()