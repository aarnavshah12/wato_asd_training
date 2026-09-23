# ASD Navigation Notes

## What the assignment asks for

- Use `/map`, `/goal_point`, and `/odom/filtered` to plan a route, then publish it on `/path`.
- Use A* as the suggested pathfinding approach: explore the grid, keep a cost for reaching each cell, and use an estimate of the remaining distance to guide the search.
- Wait for a goal, follow it, and replan when the map changes or the robot needs a new route.

## A* and Theta*

- A* moves between neighbouring cells. A cell's parent is another nearby cell, so its final path is tied to the grid directions, even when diagonal moves are allowed.
- Theta* keeps the same basic search: cost so far, estimated distance to the goal, an open list, and visited cells.
- The difference is the parent. Theta* asks whether a new cell can connect straight back to the current cell's parent. If that line is clear and cheaper, it skips the current cell as a waypoint.
- That can make the route straighter and give the controller fewer corners to follow. The line checks also take extra work, so Theta* is not automatically faster to compute or guaranteed to find the shortest possible path.

## What I implemented

- The planner checks the eight cells around the current cell. For each possible next cell, it compares a normal step with a Theta* shortcut from the current cell's parent, then keeps the cheaper valid choice.
- A shortcut is checked along the whole line. It cannot cross a blocked cell or squeeze diagonally through an obstacle corner.
- The route has costs as well as distance. Unknown space costs more than known free space, and travelling close to obstacles costs more than travelling through open space. This is why a slightly longer route may be chosen.
- The planner uses an extra safety buffer around obstacle hits. It prefers about **2.3 m** of clearance and blocks cells closer than about **1.4 m**. This buffer stays inside the planner; it does not enlarge the `/map` shown in Foxglove.
- A new map only triggers a new path if the current path is blocked. A new goal or the replanning timeout can also trigger planning. This avoids replacing a usable route on every map update.

## Why I chose it

- The car can drive across open space in a straight line, so I did not want its path to zigzag just because the map is a grid.
- The clearance buffer gives the car room to turn instead of letting a straight shortcut shave an obstacle corner.
- It is still a simple approximation of the car's width, length, and steering error. It does not rotate an exact car shape along every segment, and it can reject a start or goal placed too close to a marked obstacle.
