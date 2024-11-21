const cube = document.querySelector('.cube');
const scene = document.querySelector('.scene');
let fakeScroll = 0; // Tracks the fake scroll position for cube rotation
const fakeScrollLimit = 1000; // Number of fake scroll "pixels" before enabling real scrolling
let scrollEnabled = false; // Tracks if normal scrolling is allowed
let isInCubeSection = true; // Tracks if the user is in the cube section

// Handle scroll events to check cube section visibility
window.addEventListener('scroll', () => {
  const sceneRect = scene.getBoundingClientRect();

  // Check if the cube section is in view
//   if (sceneRect.top >= 0 && sceneRect.bottom <= window.innerHeight) {
//     isInCubeSection = true;
//   } else {
//     isInCubeSection = false;
//     scrollEnabled = true; // Allow normal scrolling outside the cube section
//   }
});

// Handle wheel events for fake scrolling
window.addEventListener(
  'wheel',
  (e) => {
    console.log(fakeScroll)
    console.log(isInCubeSection)
    fakeScroll += e.deltaY;
    fakeScroll = Math.max(0, fakeScroll);
    if (isInCubeSection && !scrollEnabled) {
      e.preventDefault(); // Prevent normal scrolling

      // Adjust the fake scroll position
      //fakeScroll += e.deltaY;

      // Rotate the cube dynamically
      const rotateX = (fakeScroll / 4) % 360;
      const rotateY = (fakeScroll / 4) % 360;
      cube.style.transform = `rotateX(${rotateX}deg) rotateY(${rotateY}deg)`;

      // Enable normal scrolling after the fake scroll threshold is reached
      if (Math.abs(fakeScroll) >= fakeScrollLimit) {
        scrollEnabled = true;
        console.log('Scroll enabled');
      }
    } 
    if (Math.abs(fakeScroll) >= fakeScrollLimit) {
        scrollEnabled = true;
        console.log('Scroll enabled');
      }else if (Math.abs(fakeScroll) <= fakeScrollLimit) {
        scrollEnabled = false;
        isInCubeSection = true;
        const rotateX = (fakeScroll / 4) % 360;
        const rotateY = (fakeScroll / 4) % 360;
        cube.style.transform = `rotateX(${rotateX}deg) rotateY(${rotateY}deg)`;
    }
    else if (isInCubeSection && scrollEnabled) {
      // Allow normal scrolling when enabled
      return;
    } else if (!isInCubeSection) {
      // Allow scrolling when not in the cube section
      return;
    }
  },
  { passive: false } // Explicitly set to non-passive
);
