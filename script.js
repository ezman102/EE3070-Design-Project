// Add event listeners for keypad buttons
document.getElementById('btn1').addEventListener('click', function() {
    // Send a request to the server to simulate pressing '1'
    fetch('/control?button=1', { method: 'POST' });
    // You can handle the response if needed
});
// Add event listeners for other keypad buttons
