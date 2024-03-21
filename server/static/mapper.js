const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

// Computed when data received, this is a global!
offsets = { x: { min: 0, max: 0 }, y: { min: 0, max: 0 }}
since_update = 0;

function scalePoint(point) {
    const scaleX = (canvas.width - 10) / (offsets.x.max - offsets.x.min); // Subtract a margin if needed
    const scaleY = (canvas.height - 10) / (offsets.y.max - offsets.y.min);

    const x = (point[0] - offsets.x.min) * scaleX + 5;
    const y = (point[1] - offsets.y.min) * scaleY + 5;

    console.log(x, y, offsets);

    return [x, y];
}

function drawPoints(spatialPoints, positionPoints) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Position points
    ctx.fillStyle = 'black';
    spatialPoints.forEach(point => {
        const [scaledX, scaledY] = scalePoint(point);
        ctx.beginPath();
        ctx.arc(scaledX, scaledY, 5, 0, 2 * Math.PI);
        ctx.fill();
    });

    // This is the start for the lines
    ctx.beginPath();
    const [firstX, firstY] = scalePoint(positionPoints[0]);
    ctx.moveTo(firstX, firstY);

    ctx.fillStyle = 'red'; // Dot
    ctx.strokeStyle = 'red'; // Line
    positionPoints.forEach((point, index) => {
        const [scaledX, scaledY] = scalePoint(point);

        // Only start drawing the line after first val
        if (index > 0) {
            ctx.lineTo(scaledX, scaledY);
        }

        ctx.stroke();

        // Dot
        ctx.beginPath();
        ctx.arc(scaledX, scaledY, 5, 0, 2 * Math.PI);
        ctx.fill();

        // Line moves from previous point to current
        ctx.beginPath();
        ctx.moveTo(scaledX, scaledY);
    });

    since_update = 0;
}

function fetchAndDraw() {
    fetch('http://localhost:3000/api/points/1?mock=true')
        .then(response => response.json())
        .then(data => {
            // Compute the points to find offset
            offsets = [...data.spatial_points, ...data.position_points].reduce((acc, point) => {
                const [x, y] = point;
            
                acc.x.min = Math.min(acc.x.min, x);
                acc.x.max = Math.max(acc.x.max, x);
                acc.y.min = Math.min(acc.y.min, y);
                acc.y.max = Math.max(acc.y.max, y);
            
                return acc;
            }, {
                // Initial vals
                x: { min: data.spatial_points[0][0], max: data.spatial_points[0][0] },
                y: { min: data.spatial_points[0][1], max: data.spatial_points[0][1] }
            });

            // Offsets stored global, now we render
            drawPoints(data.spatial_points, data.position_points);
        })
        .catch(error => console.error('Error fetching data:', error));
}

fetchAndDraw();
setInterval(fetchAndDraw, 8000);

// Handles the timer text
setInterval(function() {
    document.querySelector("#updated").innerText = since_update == 0 ? "now" : `${since_update} seconds ago..`;
    since_update++;
}, 1000);