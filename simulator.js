function Simulator()
{
	this.agents = []; // Agent[]
	this.obstacles = []; // Obstacle[]
	this.goals = []; // Vector2
	this.kdTree = new KdTree();
	
	this.timeStep = 0.25;
	
    this.defaultAgent = null; // Agent
    this.time = 0.0;
    
    this.getGlobalTime = function() {
    	return this.time;
    };
    
    this.getNumAgents = function() {
    	return this.agents.length;
    };
    
    this.getTimeStep = function() {
    	return this.timeStep;
    };

    this.setAgentPrefVelocity = function(i, velocity) {
        this.agents[i].prefVelocity = velocity;
    };
    
    this.setTimeStep = function(timeStep)
    {
        this.timeStep = timeStep;
    };
    
    this.getAgentPosition = function(i)
    {
        return this.agents[i].position;
    };
    
    this.getAgentPrefVelocity = function(i)
    {
        return this.agents[i].prefVelocity;
    };
    
    this.getAgentVelocity = function(i)
    {
        return this.agents[i].velocity;
    };
    
    this.getAgentRadius = function(i)
    {
        return this.agents[i].radius;
    };
    
    this.getAgentOrcaLines = function(i)
    {
        return this.agents[i].orcaLines;
    };

    this.addAgent = function(position)
    {
        if (this.defaultAgent == null) {
            throw new Error("no default agent");
        }

        var agent = new Agent();

        agent.position = position;
        agent.maxNeighbors = this.defaultAgent.maxNeighbors;
        agent.maxSpeed = this.defaultAgent.maxSpeed;
        agent.neighborDist = this.defaultAgent.neighborDist;
        agent.radius = this.defaultAgent.radius;
        agent.timeHorizon = this.defaultAgent.timeHorizon;
        agent.timeHorizonObst = this.defaultAgent.timeHorizonObst;
        agent.velocity = this.defaultAgent.velocity;

        agent.id = this.agents.length;
        this.agents.push(agent);

        return this.agents.length - 1;
    };

    //  /** float */ neighborDist, /** int */ maxNeighbors, /** float */ timeHorizon, /** float */ timeHorizonObst, /** float */ radius, /** float*/ maxSpeed, /** Vector2 */ velocity)
    this.setAgentDefaults = function(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius,  maxSpeed, velocity)
    {
        if (this.defaultAgent == null)
        {
        	this.defaultAgent = new Agent();
        }

        this.defaultAgent.maxNeighbors = maxNeighbors;
        this.defaultAgent.maxSpeed = maxSpeed;
        this.defaultAgent.neighborDist = neighborDist;
        this.defaultAgent.radius = radius;
        this.defaultAgent.timeHorizon = timeHorizon;
        this.defaultAgent.timeHorizonObst = timeHorizonObst;
        this.defaultAgent.velocity = velocity;
    }
    
    this.run = function(callback)
    {
    	/**
    	if (threadworkers.length == 0)
        {
            for (var block = 0; block < threadworkers.length; ++block)
            {
                threadworkers.push(new ThreadWorker(block * this.getNumAgents() / threadworkers.length, (block + 1) * this.getNumAgents() / threadworkers.length));
            }
        }
    	*/
    	
    	this.kdTree.buildAgentTree();

    	for (var i = 0; i < this.getNumAgents(); i++) {
	    	this.agents[i].computeNeighbors();
	        this.agents[i].computeNewVelocity();
    	}
    	
    	this.time += this.timeStep;
    	
    	for (var i = 0; i < this.getNumAgents(); i++) {
    		this.agents[i].update();
    	}
   
    	if (callback) {
    		callback();
    	}
    	
        /**
        var i, w;
	    httpworkers = []; // array of workers for this step

	    for (i = 0; i < this.getNumAgents(); i++) {
	        // set up new worker
	        w = new Worker( 'worker.js' );
	        w.onmessage = function ( event ) {
	            var i;

	            if (event.data === 'finished') {
	                this.finished = true;

	                for ( i = 0; i < httpworkers.length; i++ ) {
	                    if ( !simulator.httpworkers[i].finished ) {
	                        return;
	                    };
	                }

	                // survived for-loop = all workers finished
	                if ( !reachedGoal( ) ) { // another iteration?
	                	this.time += this.timeStep;
	                    simulator.run(callback);
	                };
	            };
	        };
	        
	        w.onerror = function(event){
	            throw new Error(event.message + " (" + event.filename + ":" + event.lineno + ")");
	        };
	        
	        httpworkers.push(w);
	        w.postMessage({"cmd": "step", "agentid": i});
	    };
	    
	    */
    };
    
	this.reachedGoal = function()
	{
		// Check whether all agents have arrived at their goals.
		for (var i = 0; i < this.getNumAgents (); ++i) {
			if (RVOMath.absSq (this.goals[i].minus(this.getAgentPosition(i))) > this.getAgentRadius (i) * this.getAgentRadius (i)) {
				// Agent is further away from its goal than one radius.
				return false;
			};
		}
		return true;
	};

	this.addGoals = function(goals) {
		this.goals = goals;
	};
	
    this.getGoal = function(goalNo)
    {
        return this.goals[goalNo];
    };
	
    this.addObstacle = function( /** IList<Vector2> */ vertices)
    {
        if (vertices.length < 2)
        {
            return -1;
        }

        var obstacleNo = this.obstacles.length;

        for (var i = 0; i < vertices.length; ++i)
        {
            var obstacle = new Obstacle();
            obstacle.point = vertices[i];
            if (i != 0)
            {
                obstacle.prevObstacle = this.obstacles[this.obstacles.length - 1];
                obstacle.prevObstacle.nextObstacle = obstacle;
            }
            if (i == vertices.length - 1)
            {
                obstacle.nextObstacle = this.obstacles[obstacleNo];
                obstacle.nextObstacle.prevObstacle = obstacle;
            }
            obstacle.unitDir = RVOMath.normalize(vertices[(i == vertices.length - 1 ? 0 : i + 1)].minus(vertices[i]));

            if (vertices.length == 2)
            {
                obstacle.isConvex = true;
            }
            else
            {
                obstacle.isConvex = (RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)], vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
            }

            obstacle.id = this.obstacles.length;

            this.obstacles.push(obstacle);
        }

        return obstacleNo;
    }

    this.processObstacles = function()
    {
        this.kdTree.buildObstacleTree();
    };

    var queryVisibility = function(/** Vector2 */ point1, /** Vector2 */ point2, /** float */ radius)
    {
        return this.kdTree.queryVisibility(point1, point2, radius);
    };

    /**
    var getNumObstacleVertices = function()
    {
        return this.obstacles.length;
    }

    var getObstacleVertex = function(vertexNo)
    {
        return this.obstacles[vertexNo].point;
    }

    var getNextObstacleVertexNo = function(vertexNo)
    {
        return this.obstacles[vertexNo].nextObstacle.id;
    }

    var getPrevObstacleVertexNo = function(vertexNo)
    {
        return this.obstacles[vertexNo].prevObstacle.id;
    }
    */
}

Simulator.instance = new Simulator(); // not a real singleton