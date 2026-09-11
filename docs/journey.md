# The Journey

**Team:** Razan Shalabi · Shatha Abualrub · Lara Daifallah · Ghada Swalha
**Instructor:** Wasel Ghanem
**University:** Birzeit University

This is the story of how Toots came together — from an empty table to a robot that solves an 8×8 maze on its own. The full day-to-day board is on [Trello](https://trello.com/invite/b/69e2683745c4a1b255d148a3/ATTI001aa2c3a295a99a65e10da04925d9e88B10C430/interface-project) if you want to see every step.

## Let the Journey Begin

It started with a pile of components arriving — motors, sensors, the ESP32, all still in their static bags. Once everything was in hand, the team sat down together to start planning how it would all fit together and work.

## 3D Printing the Chassis

<img src="../media/fusion-design.png" width="400">

The chassis was modeled in Fusion 360 before ever touching a printer — laid out to fit within the maze cell constraints while staying light enough not to load down the N20 motors.

<img src="../media/chassis.jpg" width="400">

The printed pieces came together into the final frame. The chassis was printed for free at Blue Dome, keeping the build's cost down.

## Maze Simulation

Before the physical robot ever moved through a real maze, the floodfill (BFS) solving logic was tested in simulation — mapping out the shortest path across an 8×8 grid to confirm the algorithm actually worked before trusting it to real hardware.

## Bringing It All Together

<img src="../media/toots.jpeg" width="400">

With the chassis built, components wired, and the algorithm proven in simulation, everything came together into Toots — a fully autonomous maze-solving robot, wall-following with three ToF sensors, tracking its movement with wheel encoders, and finding its way to the center of the maze without any human input.

## Documentation Along the Way

The team kept detailed reports throughout the build, including a Phase 1 virtual logic & structural innovation report and a 3D printing hardware report — part of the full record kept on the [Trello board](https://trello.com/invite/b/69e2683745c4a1b255d148a3/ATTI001aa2c3a295a99a65e10da04925d9e88B10C430/interface-project).

---

Want the blow-by-blow, including early struggles and every design decision? The full Trello board is linked above.
