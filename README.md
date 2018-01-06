RVO2 port to Javascript
=======================

RVO2 Library: Reciprocal Collision Avoidance for Real-Time Multi-Agent Simulation.

This is an **alpha release** of a RVO2 port from the [C# version](http://rvo-unity.chezslan.fr) to Javascript, only for
research purposes and still not intended to be used in production environments. Paper with full details as well as C++
code can be found at the [original authors' site](http://gamma.cs.unc.edu/RVO2).

See the **[online demo](http://guidogarcia.net/demos/rvo2-js)**.

Basic Usage
-----------

### Import core javascript files

~~~~html
	<script type="text/javascript" src="lib/common.js"></script>
	<script type="text/javascript" src="lib/agent.js"></script>
	<script type="text/javascript" src="lib/kdtree.js"></script>
	<script type="text/javascript" src="lib/simulator.js"></script>
~~~~

### Setup your custom scenario

~~~~js
	var simulator = Simulator.instance;

	var setupScenario = function(simulator)	{
			// Specify global time step of the simulation.
			simulator.setTimeStep(0.25);

			// Specify default parameters for agents that are subsequently added
			simulator.setAgentDefaults(
					200, // neighbor distance (min = radius * radius)
					30, // max neighbors
					600, // time horizon
					600, // time horizon obstacles
					5, // agent radius
					10.0, // max speed
					new Vector2(2, 2) // default velocity
				);

			// Put 9 agents in a circle
			for (var i=0; i<9; i++) {
				var angle = i * (2 * Math.PI) / 9;
				var x = Math.cos(angle) * 200;
				var y = Math.sin(angle) * 200;
				simulator.addAgent(new Vector2 (x, y));
 			}

			// Create agent targets
			var goals = [];
			for (var i = 0; i < simulator.getNumAgents (); ++i) {
				goals.push(simulator.getAgentPosition(i).scale(-1));
			}
			simulator.addGoals(goals);
		}
~~~~

### How to add one obstacle

~~~~js
	// Add obstacle, specifying vertices in counterclockwise order.
	var vertices = [];

	vertices.push(new Vector2 (-40.0, -90.0));
	vertices.push(new Vector2 (40.0, -90.0));
	vertices.push(new Vector2 (40.0, -10.0));
	vertices.push(new Vector2 (-40.0, -10.0));

	simulator.addObstacle (vertices);

	// Process obstacles so that they are accounted for in the simulation.
	simulator.processObstacles ();
~~~~

### Run the simulation

~~~~js
	var interval;
	var run = function() {
		setupScenario(simulator);

		var step = function() {
			setPreferredVelocities(simulator);
			simulator.run();

			// here you can do whatever you need (i.e. draw the agents)

			if (simulator.reachedGoal()) {
				clearInterval(interval);
			}
		}

		interval = setInterval(step, 10);
	}
~~~~

### Dynamically adapt agent velocities

~~~~js
	var setPreferredVelocities = function(simulator) {
		for (var i = 0; i < simulator.getNumAgents (); ++i) {
			if (RVOMath.absSq(simulator.getGoal(i).minus(simulator.getAgentPosition(i))) < 0) {
				simulator.setAgentPrefVelocity (i, new Vector2 (0, 0));
			} else {
				// Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
				simulator.setAgentPrefVelocity(i, RVOMath.normalize(simulator.getGoal(i).minus(simulator.getAgentPosition(i))));
			}
		}
	}
~~~~

License
-------

Copyright (c) 2008-10 University of North Carolina at Chapel Hill. All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation for educational, research, and non-profit purposes, without fee, and without a written agreement is hereby granted, provided that the above copyright notice, this paragraph, and the following four paragraphs appear in all copies.

Permission to incorporate this software into commercial products may be obtained by contacting the University of North Carolina at Chapel Hill.

This software program and documentation are copyrighted by the University of North Carolina at Chapel Hill. The software program and documentation are supplied "as is", without any accompanying services from the University of North Carolina at Chapel Hill or the authors. The University of North Carolina at Chapel Hill and the authors do not warrant that the operation of the program will be uninterrupted or error-free. The end-user understands that the program was developed for research purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR ITS EMPLOYEES OR THE AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
