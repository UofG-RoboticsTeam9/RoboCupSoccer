# ROBOCUP SOCCER USING NAO ROBOTS(TDP PROJECT)
# Introduction
The purpose of this project is to acquire and develop important Engineering principles relevant to the Robotics and Artificial Intelligence field. Alongside these principles, communication, teamwork and project management are also crucial skills that are required to correctly execute the project. Furthermore, cost estimations and calendar planning were administered to replicate a real work environment and to gauge the deviation from the initial costs/calendar.
The objective of the project is to create/design two teams of Humanoid NAO robots where each team consists of four players including a striker, defender and a goalkeeper. The idea is to imitate real life Human Football competition. Since the kinematics, dynamics and control physics of the NAO robots closely represent those of Humans, tactics and motions inspired by Real life football such as Premier League and FIFA was incorporated to further improve and emulate Human performance. The end goal was to design/implement control dynamics and physics that allows the robots to move and perform motions intelligently with the intent of following the rules of Soccer in a simulated environment. 
This problem was tackled by evaluating the ideal framework that allows convenient but also effective implementation of design ideas. ROS and MATLAB were the two primary choices considered by the team. However, each framework had its advantages and disadvantages; meaning, the team weren’t comfortable with either choice as it affected the harmony within the group. Webots was then introduced as an alternate solution that retains the benefits of the two primary choices. Unlike ROS, this framework is accessible through windows and MACOS; it also provides a more versatile support for the NAO robots ranging from control libraries to simulated environments. This allowed the team to focus on developing choreographed motions and intelligent behavioural algorithms with the use of Python as the main programming language.
# Methodology
The proposed system requires multiple stages to be implemented sequentially with each phase split between team members according to individual specialties. Meetings were held to discuss the system model regarding the control system, motion and path planning. The initial model consists of a flowchart containing states and requirements of the project. The first phase involved the design and implementation of a control system capable of supporting the NAO robot’s movements with respect to physics and gravity of the environment. The second phase focused on input perception of the environment. It’s important to emulate how human beings perceive the environments in order to replicate the behaviour appropriately. The idea behind the second stage was to perceive the environment and other robots using the sensors included with the NAO robots for the purpose of computing an output which makes use of the control dynamics to perform motions. The final stage introduces path planning algorithms capable of administering intelligence and autonomous behaviour into the NAO robots by way of programming [2].
Each stage was to be planned and executed on a step-by-step bases because every stage requires the previous stage to work for that stage to operate. For example, without the control system phase completed, the motions stage wouldn’t work. The same can be said about the behavioural aspect since the robot requires motion to act on a change in environment. The system model can be seen on figure 
# Modelling
![image](https://user-images.githubusercontent.com/84370257/179970771-39ecd0ee-466d-4bbd-ba92-8f4ab8f844f0.png)


Contributors:
Talha Enes Ayranci
Hayatu Abdullahi
Omkar Padwal
Ankai Zhang
Jing Shi
Yuzhong Ding
