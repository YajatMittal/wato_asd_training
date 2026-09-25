# WATonomous ASD Admissions Assignment

## My solution

The robot can drive itself to any point you click in Foxglove while avoiding the obstacles in the sim. I split it into the four nodes from the assignment:

```
/lidar -> costmap -> /costmap -> map_memory -> /map -> planner -> /path -> control -> /cmd_vel
```

Map memory, the planner and control also listen to `/odom/filtered` for the robot's position, and the planner gets its goal from `/goal_point`.

### Costmap

Every lidar scan gets turned into a fresh 30 m x 30 m grid (0.1 m cells) centred on the lidar. For each beam I walk along it with Bresenham's line algorithm and mark the cells it passes through as free. Then I mark where it hit as an obstacle and inflate around it with the linear formula from the assignment, `cost = 100 * (1 - d / r)`.

A few things I did differently from the example:

- Cells start out unknown (-1) instead of free. A cell only becomes free if a beam actually went through it, so whatever is hidden behind an obstacle stays unknown. Otherwise the robot would claim to know there's free space behind walls.
- The inflation radius is 2 m instead of 1 m. The robot is 2 m long and about 1.4 m wide with the wheels, so 1 m doesn't leave much room once you account for the robot's own size. The planner treats anything within about 1 m of an obstacle as a wall, and the rest of the 2 m as "you can go here, but you'd rather not".
- The inflation pattern is worked out once at startup and then just stamped around each hit, so the per-scan loop doesn't do any square roots.
- Scans with no returns at all get ignored. Gazebo sends a few of these right after it starts (every beam is inf because the world hasn't loaded yet). Before I caught that, the first map said everything was free and the robot drove straight at the cylinder.

### Map memory

This keeps a 40 m x 40 m map of everything the robot has seen so far. Like the assignment suggests, it only merges in a new costmap once the robot has moved 1.5 m (checked on a 1 s timer). The very first costmap is the exception, so the planner has a map to work with right away. The map is published as transient local, so the planner still gets it even if it starts up late.

To merge, I loop over the map cells and look up which costmap cell is underneath each one, instead of pushing costmap cells into the map. That way rotating the costmap can't leave holes, so I didn't need the costmap to be finer than the map.

Two things made a big difference here. The first is timing: each costmap is placed using where the robot was when that scan was taken (interpolated from the odometry), not where it is when the timer goes off. I checked this by spinning the robot in place. Matched to the scan time, the lidar hits land within about 1 cm of the real walls, but being off by just 0.1 s puts them over 10 cm off, and it gets worse the further away the obstacle is.

The second is that a cell keeps the higher of its old and new cost instead of being overwritten. I started with the "new data overwrites old" rule from the assignment, but the map ended up with rings and bumps around the obstacles. It turns out a beam that skims past the far side of an obstacle marks cells as free even though they're right next to it, and overwriting wiped out the inflation from earlier scans that did see that side. Nothing in this world moves, so keeping the higher cost is safe, and free space still replaces unknown. The map came out clean after that change.

### Planner

The planner runs A* on the global map using the `CellIndex`, `AStarNode` and `CompareF` structs from the assignment. The grid has a fixed size, so I used plain arrays for the g-scores and parent links instead of hash maps.

- It's 8-connected and uses octile distance as the heuristic, which never overestimates.
- Stepping into a cell costs its length times `1 + 3 * cost / 100`, so the path stays in the middle of gaps instead of scraping along the edge of the inflation.
- Cells with a cost of 50 or more are off limits. Unknown cells are allowed with a small penalty, so it will plan through areas it hasn't seen yet and fix the plan as the map fills in.
- It won't cut diagonally past the corner of an obstacle.
- If the robot ends up inside the inflated area it's allowed to walk back out, and if the goal is too close to an obstacle it heads for the nearest free spot within 1.5 m instead.

The state machine is the two states from the assignment: waiting for a goal, and waiting for the robot to reach it. It replans every time a new map comes in, replans if the robot hasn't gotten any closer in 10 s, and gives up (stopping the robot) if there's no path or it's been more than 2 minutes. The goal counts as reached within 0.5 m.

### Control

This is pure pursuit at 10 Hz with a 1.5 m lookahead, driving at 0.5 m/s. The controller keeps track of how far along the path it is and only lets that move forward, so a path that comes back near itself can't make it skip ahead. On top of the basic algorithm:

- If the lookahead point is more than about 57° off to the side or behind, it turns on the spot first. It's diff drive so it can, and pure pursuit behaves badly when the target is behind you.
- It slows down over the last 1.5 m so it doesn't overshoot, and stops within 0.25 m of the end.
- If a turn would be sharper than the max turn rate (1 rad/s), it slows down instead of just capping the turn rate. That keeps it on the same arc instead of swinging wide.
- When it's done it sends one stop command and then goes quiet, so the teleop panel still works.

### Results

In the sim it made it to every reachable goal I tried, including getting behind the big cylinder from the starting position and crossing the map between several obstacles. It usually stopped about 0.25 m from the goal, and the robot's body never got closer than about 0.7 m to an obstacle. For a goal outside the walls, it drives until it sees the wall, then gives up and stops.

All the numbers I tuned are in each package's `config/params.yaml`.

### Running it

```
./watod build
./watod up
```

Then connect Foxglove to the bridge (the port shows up in the logs), import `config/wato_asd_training_foxglove_config .json`, pick "Publish point" in the 3D panel and click wherever you want the robot to go.

### What I'd change next

- Keeping the higher cost only works because nothing moves. For moving obstacles I'd store obstacle hits and free space in the map instead, and inflate the whole map after each update.
- Pure pursuit follows the lidar's position (that's what `/odom/filtered` gives), but the robot actually turns around its rear axle, 1.3 m behind the lidar. On tight turns the back cuts in a bit, so following the axle point would be more accurate.
- Some smoothing on the A* path, since it's made of 45° steps and pure pursuit is doing all of the smoothing right now.

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/
