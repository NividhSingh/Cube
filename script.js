const cube = document.querySelector('.cube');

window.addEventListener('scroll', () => {
  const scrollY = window.scrollY;
  
  // Calculate rotation angles based on scroll position
  const rotateX = scrollY * 0.5; // Adjust the multiplier to control rotation speed
  const rotateY = scrollY * 0.5;

  // Apply rotation
  cube.style.transform = `rotateX(${rotateX}deg) rotateY(${rotateY}deg)`;
});
