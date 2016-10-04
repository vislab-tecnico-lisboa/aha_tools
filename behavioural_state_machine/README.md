# behavioural_state_machine

To use package with real robot:

1. Edit connect_node.py to select DB ip.
2. Run connect_node.py to read from MondoDB and publish events on a topic.
3. Run main script of the state machine.


To use package in simulation:

1. $ mongod
2. $ mongo
3. $ roslaunch vizzy_launch vizzy_simulation.launch
4. $ rviz
5. Edit connect_node.py to select DB ip (in this case could be empty).
6. Run connect_node.py to read from MondoDB and publish events on a topic.
7. Run main script of the state machine.
8. Add events in mongo to simulate. An example of event is

	db.GestureBuildEvents.insert(
	   {
	     timeStamp: "23:01",
	     confidence: 0.91,
	     gesture: "come here"
	   }
	)



