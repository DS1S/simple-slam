const canvas = document.getElementById('map_canvas');
const ctx = canvas.getContext('2d');

// Computed when data received, this is a global!
since_update = 0;

// Chart data
const spatial_points = [];
const position_points = [];
const chart_data = {
    datasets: [
        {
            label: 'Spatial Points',
            data: spatial_points,
            backgroundColor: 'rgb(32, 32, 32)',
        },
        {
            label: 'Position Points',
            data: position_points,
            showLine: true,
            backgroundColor: 'rgb(255, 0, 0)',
        }
    ]
};

// Chart options
Chart.defaults.font.size = 16;
const options = {
    animation: {
        duration: 0
    },
    elements: {
        point: {
            radius: 6,
        },
        line: {
            borderColor: 'rgb(255, 0, 0)',
            borderWidth: 1
        }
    },
    scales: {
        x: {
            title: {
                display: true,
                text: 'X'
            },
        },
        y: {
            title: {
                display: true,
                text: 'Y',
            },
        }
    },
    plugins: {
        zoom: {
            zoom: {
                wheel: {
                    enabled: true
                }
            },
            pan: {
                enabled: true,
            }
        }
    }
};

const chart = new Chart(canvas, {
    type: 'scatter',
    data: chart_data,
    options: options
});

function fetchAndDraw() {
    fetch('http://localhost:3000/api/points/1?mock=true')
        .then(response => response.json())
        .then(data => {        
            spatial_points.length = 0;
            data.spatial_points.forEach((element) => {
                spatial_points.push({x: element[0], y: element[1]})
            });
            position_points.length = 0;
            data.position_points.forEach((element) => {
                position_points.push({x: element[0], y: element[1]})
            });
            chart.update();
        });
    since_update = 0;
}

fetchAndDraw();
setInterval(fetchAndDraw, 8000);

// Handles the timer text
setInterval(function() {
    document.querySelector("#updated").innerText = since_update == 0 ? "now" : `${since_update} seconds ago..`;
    since_update++;
}, 1000);
