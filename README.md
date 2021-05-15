**Launching the Simulation**

Download the folder from this Github: [https://github.com/alexcn3/swarmformation](https://github.com/alexcn3/swarmformation)

When you've set it up in your ROS environment, open one tab in terminal and navigate to the swarmformation folder and run these two commands:

```source /home/{user}/{catkin_package_folder}/devel/setup.bash```
```roslaunch swarm_formation_py stdr_test.launch```

This launches the server. Now open up a second tab in the same folder and run these three commands:

```source /home/{user}/{catkin_package_folder}/devel/setup.bash```
```for i in `seq 1 {number of robots}`; do rosrun stdr_robot robot_handler /home/{user}/{catkin_package_folder}/src/swarm_formation_py/robot/squarerobot.xml `$i/25 | bc -l` `$i/25 | bc -l` 0; done```
```roslaunch stdr_gui stdr_gui.launch```

The first command links the resources necessary. The second command adds square simulations of robots to the server. The final command launches the GUI for viewing these simulations. 

To run the code for behavior, open another tab in the terminal. Start off running this command as usual:

```source /home/{user}/{catkin_package_folder}/devel/setup.bash```

If you'd like to run the move_test.py to make sure the simulation's move mechanism is still working, run this command:

```rosrun swarm_formation_py move_test.py '{robot name}' '{pointl1}' '{point2}' ... '{pointlx}'```

This should result in the robot you pass in moving between the points passed in. 

**Demo**

[https://vimeo.com/549508731](https://vimeo.com/549508731) &lt;--- random movement between two given points for each of the 10 robots

[https://vimeo.com/549508740](https://vimeo.com/549508740) &lt;--- movement between the center of the grid and the outskirts for each of the 10 robots

If you'd like to test the swarm formation algorithm, run this command *framework is there but not fully functional:

rosrun swarm_formation_py brainstorm.py '{robot name}' '{goal1}' '{goal2}' ... '{goalx}'

**Going forward...**



*   integrating the move_test code fully into the brainstorm,py. as of now, the robot spins constantly without ever moving when move() is called in brainstorm.py even though the same logic works just fine in move_test.py
*   working on communication between two or more robots so that goal_manager and broadcast_component can be properly implemented

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
