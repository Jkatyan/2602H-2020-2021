**Documentation:**
-
**Drive Movements**

To create a path, use `gen("Name", y_in, x_in, theta_deg);` *Y and X values cannot be negative.*

To run a path, use `run("Name", false);` *True to reverse a path.*

To finish a path, use `finishRun("Name");` *This deletes the path. To keep a path, do not finish it.*

**YOU CAN ONLY HAVE 5 PATHS ALIVE AT ONCE.** Also to prevent delays caused by path generation, generate paths while running others. Straight (y) movements are more accurate than (x, y) movements.

To turn, use `profilePosTurn(deg, reset, max_vel);` *Example params: (90, true, 150). Set to true, unless you're turning for a 2nd or 3rd time before aligning at a goal.*

**Intake and Conveyor States**

Set state using `setStateConveyor(state)` or `setStateIntakes(state)`

States: 0- stopped. 1- spinning in. 2- spinning out. 3- indexing *conveyor only*
