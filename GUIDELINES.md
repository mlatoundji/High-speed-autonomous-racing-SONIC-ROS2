Objective: In the era of AI-assisted coding, our focus shifts from merely writing functional code to system integration and engineering validation. You will be evaluated on how well your components work together as a robust system.


Project D: High-speed autonomous racing
Scenario: On a race track, the robot (preferably using an Ackermann steering model) must complete laps as quickly as possible without colliding with track boundaries.

Challenges:

Manage the latency between perception and execution at high speeds.
Implement advanced controllers like Pure Pursuit or Model Predictive Control (MPC), as default Nav2 plugins may be conservative.
Calculate and follow the “racing line” rather than just the track centerline.
Deliverables:

A custom controller plugin for Nav2 or a standalone high-speed control node.
A comparison of lap times under conservative and aggressive parameter tuning.
Bonus: Analyze the impact of odometry drift by introducing artificial noise into it and plot the resulting degradation in lap times.


Each team must provide the following:

GitHub repository: - A clean ROS 2 workspace structure.
Custom nodes, launch files, Gazebo world files, etc.
A comprehensive README.md with installation, execution instructions, etc.
Project report:
Max 10 pages in PDF format.
Content: team members and task distribution, system architecture (including a detailed diagram), and a “lessons learned” section detailing challenges and solutions.
Final presentation:
Session 18: 10-minute presentation + 10-minute Q&A, per team.