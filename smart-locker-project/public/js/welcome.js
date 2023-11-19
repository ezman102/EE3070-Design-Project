// welcome.js
function showWelcomePage() {
    const contentDiv = document.querySelector('.content');
    const greeting = getGreetingByTime();

    contentDiv.innerHTML = `
        <div class="welcome-container">
            <h2 class="welcome-title">${greeting}, ${displayName}!</h2>
            <p class="welcome-message">Your locker number :  [ ${lockerNumber} ]</p>
            <img src="../images/locker.jpg" alt="locker" class="welcome-image">
        </div>
    `;
}

function getGreetingByTime() {
    const hour = new Date().getHours();
    if (hour < 12) return 'Good Morning';
    if (hour < 18) return 'Good Afternoon';
    return 'Good Evening';
}