  # tyrolics_refbox
    author: Martin Sereinig (?)   
  Additional to the atWorkComanderRefbox:



  ## HowTo

  ### Starting the Refbox

  1. Start the __core__ and __com__ components:
  ```
  roslaunch tyrolics_refbox atwork_commander.launch
  ```


  2. Start the Robot launch file:
  ```
  roslaunch tyrolics_refbox leobot_robot.launch
  ```

  3. Generate a task using the CLI:
  ```
  roslaunch atwork_commander generate.launch task:=<task to generate>
  ```
  4. Wait for robots to register
  5. Start the task execution using the CLI:
  ```
  roslaunch atwork_commander start.launch
  ```

  For testing the Refbox without any robot a fake robot may be used using the **example_robot**:
  `roslaunch atwork_commander example_robot.launch`

  ### Configurations

  - All launch files share the __refbox__ parameter to specify the Refbox to connect
    to. Multiple Refboxes can be started on the same PC using multiple namespaces.
  - When more information is needed, the __verbose__ parameter can be enabled for
    more verbose logging.
  - If a task needs to be sent to only some registered robots, the robots can be
    specified in the __start__ command using the robots parameter in the format
    "<team_name>/<robot_name>"

  ### Runtime control

  - the __forward__ launch file enables manual state change from *PREPARATION* to *EXECUTION*:
  ```
  roslaunch atwork_commander forward.launch
  ```
  - the __stop__ launch file enables stopping of the currently running task:
  ```
  roslaunch atwork_commander stop.launch
  ```
