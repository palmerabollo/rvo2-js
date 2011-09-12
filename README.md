RVO2 port to Javasript
----------------------

'''RVO2 Library: Reciprocal Collision Avoidance for Real-Time Multi-Agent Simulation'''.

This is an '''alpha release''' of a RVO2 port from the [http://rvo-unity.chezslan.fr C# version] to Javascript, only for
research purposes and still not intended to be used in production environments.

Paper with full details as well as C++ code can be found at the [http://gamma.cs.unc.edu/RVO2 original authors' site].

Usage
-----

### Import javascript files

~~~~
	<script type="text/javascript" src="lib/common.js"></script>
	<script type="text/javascript" src="lib/agent.js"></script>
	<script type="text/javascript" src="lib/kdtree.js"></script>
	<script type="text/javascript" src="lib/simulator.js"></script>
~~~~

=== Setup your custom scenario ===

~~~~
	var setupScenario = function(simulator)	{
			// Specify global time step of the simulation.
			simulator.setTimeStep(0.25);
			
			// Specify default parameters for agents that are subsequently added.
			var velocity = new Vector2(2, 2);
			simulator.setAgentDefaults (200.0, 100, 1000, 1000, 5, 5.0, velocity);
			
			for (var i=0; i<9; i++) {
				var angle = i * (2 * Math.PI) / 9;
				var x = Math.cos(angle) * 240;
				var y = Math.sin(angle) * 240;
				simulator.addAgent(new Vector2 (x, y));
 			}
			
			// Create goals
			var goals = [];
			for (var i = 0; i < simulator.getNumAgents (); ++i) {
				goals.push(simulator.getAgentPosition(i).scale(-1));
			}
			simulator.addGoals(goals);
			
			// Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
			var vertices = [];
			/**
			vertices.push(new Vector2 (-40.0, -90.0));
			vertices.push(new Vector2 (40.0, -90.0));
			vertices.push(new Vector2 (40.0, -10.0));
			vertices.push(new Vector2 (-40.0, -10.0));
			*/
			simulator.addObstacle (vertices);
			
			// Process obstacles so that they are accounted for in the simulation.
			simulator.processObstacles ();
		}
~~~~

### Run the simulator

~~~~
	var simulator = Simulator.instance;
		
	var interval;
	var run = function() {
		setupScenario (simulator);
			
		var step = function() {
			setPreferredVelocities(simulator);
			simulator.run();
				
			if (simulator.reachedGoal()) {
				clearInterval(interval);
			}
		}
			
		interval = setInterval(step, 5);
	}
~~~~

### Adapt velocities

~~~~		
	var setPreferredVelocities = function(simulator) {
		for (var i = 0; i < simulator.getNumAgents (); ++i) {
			if (RVOMath.absSq(simulator.getGoal(i).minus(simulator.getAgentPosition(i))) < simulator.getAgentRadius(i) * simulator.getAgentRadius(i)) {
				// Agent is within one radius of its goal, set preferred velocity to zero
				simulator.setAgentPrefVelocity (i, new Vector2 (0.0, 0.0));
			} else {
				// Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
				simulator.setAgentPrefVelocity(i, RVOMath.normalize (simulator.getGoal(i).minus(simulator.getAgentPosition(i))));
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
