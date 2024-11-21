const cube = document.querySelector('.cube');

window.addEventListener('scroll', () => {
  const scrollY = window.scrollY;

  // Calculate rotation angles based on scroll position
  const rotateX = scrollY * 0.3; // Adjust to control rotation speed
  const rotateY = scrollY * 0.4; // Adjust for different axes
  const rotateZ = scrollY * 0.2; // Add Z-axis spin for balancing effect

  // Apply transformation
  cube.style.transform = `rotateX(${rotateX}deg) rotateY(${rotateY}deg) rotateZ(${rotateZ}deg) translateY(-100px)`;
});
