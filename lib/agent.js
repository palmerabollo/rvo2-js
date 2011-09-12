function Agent() {  
	this.agentNeighbors = []; //  new List<KeyValuePair<float, Agent>>();
	this.maxNeighbors = 0;
	this.maxSpeed = 0.0;
	this.neighborDist = 0.0;
	this.newVelocity; // Vector 2
	this.obstaclNeighbors = []; // new List<KeyValuePair<float, Obstacle>>();
	this.orcaLines = []; // new List<Line>();
	this.position; // Vector2
	
	this.prefVelocity; // Vector2
	
	this.radius = 0.0;
	this.timeHorizon = 0.0;
	this.timeHorizonObst = 0.0;
	this.velocity; // Vector2
	this.id = 0;

    this.computeNeighbors = function()
    {
        this.obstaclNeighbors = [];
        var rangeSq = RVOMath.sqr(this.timeHorizonObst * this.maxSpeed + this.radius);
        Simulator.instance.kdTree.computeObstacleNeighbors(this, rangeSq);

        this.agentNeighbors = [];
        if (this.maxNeighbors > 0)
        {
            rangeSq = RVOMath.sqr(this.neighborDist);
            Simulator.instance.kdTree.computeAgentNeighbors(this, rangeSq);
        }
    };

    /* Search for the best new velocity. */
    this.computeNewVelocity = function()
    {	
        orcaLines = [];

        invTimeHorizonObst = 1.0 / this.timeHorizonObst;

        /* Create obstacle ORCA lines. */
        for (var i = 0; i < this.obstaclNeighbors.length; ++i)
        {
            obstacle1 = this.obstaclNeighbors[i].value;
            obstacle2 = obstacle1.nextObstacle;

            relativePosition1 = obstacle1.point.minus(this.position);
            relativePosition2 = obstacle2.point.minus(this.position);

            /* 
             * Check if velocity obstacle of obstacle is already taken care of by
             * previously constructed obstacle ORCA lines.
             */
            alreadyCovered = false;

            for (var j = 0; j < orcaLines.length; ++j)
            {
                if (RVOMath.det(relativePosition1.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVOEPSILON && RVOMath.det(relativePosition2.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVOEPSILON)
                {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
            {
                continue;
            }

            /* Not yet covered. Check for collisions. */

            distSq1 = RVOMath.absSq(relativePosition1);
            distSq2 = RVOMath.absSq(relativePosition2);

            radiusSq = RVOMath.sqr(this.radius);

            obstacleVector = obstacle2.point.minus(obstacle1.point);
            s = relativePosition1.scale(-1).multiply(obstacleVector) / RVOMath.absSq(obstacleVector); //  (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector);
            distSqLine = RVOMath.absSq(relativePosition1.scale(-1).minus(obstacleVector.scale(s))); // RVOMath.absSq(-relativePosition1 - s * obstacleVector);

            var line = new Line();

            if (s < 0 && distSq1 <= radiusSq)
            {
                /* Collision with left vertex. Ignore if non-convex. */
                if (obstacle1.isConvex)
                {
                    line.point = new Vector2(0, 0);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y, relativePosition1.x));
                    orcaLines.push(line);
                }
                continue;
            }
            else if (s > 1 && distSq2 <= radiusSq)
            {
                /* Collision with right vertex. Ignore if non-convex 
                 * or if it will be taken care of by neighoring obstace */
                if (obstacle2.isConvex && RVOMath.det(relativePosition2, obstacle2.unitDir) >= 0)
                {
                    line.point = new Vector2(0, 0);
                    line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y, relativePosition2.x));
                    orcaLines.push(line);
                }
                continue;
            }
            else if (s >= 0 && s < 1 && distSqLine <= radiusSq)
            {
                /* Collision with obstacle segment. */
                line.point = new Vector2(0, 0);
                line.direction = obstacle1.unitDir.scale(-1);
                orcaLines.push(line);
                continue;
            }

            /* 
             * No collision.  
             * Compute legs. When obliquely viewed, both legs can come from a single
             * vertex. Legs extend cut-off line when nonconvex vertex.
             */

            var leftLegDirection, rightLegDirection;

            if (s < 0 && distSqLine <= radiusSq)
            {
                /*
                 * Obstacle viewed obliquely so that left vertex
                 * defines velocity obstacle.
                 */
                if (!obstacle1.isConvex)
                {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle2 = obstacle1;

                leg1 = Math.sqrt(distSq1 - radiusSq);
                leftLegDirection = (new Vector2(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1);
                rightLegDirection = (new Vector2(relativePosition1.x * leg1 + relativePosition1.y * this.radius, -relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1);
            }
            else if (s > 1 && distSqLine <= radiusSq)
            {
                /*
                 * Obstacle viewed obliquely so that
                 * right vertex defines velocity obstacle.
                 */
                if (!obstacle2.isConvex)
                {
                    /* Ignore obstacle. */
                    continue;
                }

                obstacle1 = obstacle2;

                leg2 = Math.sqrt(distSq2 - radiusSq);
                leftLegDirection = (new Vector2(relativePosition2.x * leg2 - relativePosition2.y * this.radius, relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2);
                rightLegDirection = (new Vector2(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2);
            }
            else
            {
                /* Usual situation. */
                if (obstacle1.isConvex)
                {
                    leg1 = Math.sqrt(distSq1 - radiusSq);
                    leftLegDirection = (new Vector2(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1);
                }
                else
                {
                    /* Left vertex non-convex; left leg extends cut-off line. */
                    leftLegDirection = obstacle1.unitDir.scale(-1);
                }

                if (obstacle2.isConvex)
                {
                    leg2 = Math.sqrt(distSq2 - radiusSq);
                    rightLegDirection = (new Vector2(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2);
                }
                else
                {
                    /* Right vertex non-convex; right leg extends cut-off line. */
                    rightLegDirection = obstacle1.unitDir;
                }
            }

            /* 
             * Legs can never point into neighboring edge when convex vertex,
             * take cutoff-line of neighboring edge instead. If velocity projected on
             * "foreign" leg, no constraint is added. 
             */

            leftNeighbor = obstacle1.prevObstacle;

            isLeftLegForeign = false;
            isRightLegForeign = false;

            if (obstacle1.isConvex && RVOMath.det(leftLegDirection, leftNeighbor.unitDir.scale(-1)) >= 0.0)
            {
                /* Left leg points into obstacle. */
                leftLegDirection = leftNeighbor.unitDir.scale(-1);
                isLeftLegForeign = true;
            }

            if (obstacle2.isConvex && RVOMath.det(rightLegDirection, obstacle2.unitDir) <= 0.0)
            {
                /* Right leg points into obstacle. */
                rightLegDirection = obstacle2.unitDir;
                isRightLegForeign = true;
            }

            /* Compute cut-off centers. */
            leftCutoff = obstacle1.point.minus(this.position).scale(invTimeHorizonObst);
            rightCutoff = obstacle2.point.minus(this.position).scale(invTimeHorizonObst);
            cutoffVec = rightCutoff.minus(leftCutoff);

            /* Project current velocity on velocity obstacle. */

            /* Check if current velocity is projected on cutoff circles. */
            t = obstacle1 == obstacle2 ? 0.5 : this.velocity.minus(leftCutoff).multiply(cutoffVec) / RVOMath.absSq(cutoffVec);
            tLeft = this.velocity.minus(leftCutoff).multiply(leftLegDirection);
            tRight = this.velocity.minus(rightCutoff).multiply(rightLegDirection);

            if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0))
            {
                /* Project on left cut-off circle. */
                var unitW = RVOMath.normalize(this.velocity.minus(leftCutoff));

                line.direction = new Vector2(unitW.y, -unitW.x);
                line.point = leftCutoff.plus(unitW.scale(this.radius * invTimeHorizonObst));
                orcaLines.push(line);
                continue;
            }
            else if (t > 1.0 && tRight < 0.0)
            {
                /* Project on right cut-off circle. */
                var unitW = RVOMath.normalize(this.velocity.minus(rightCutoff));

                line.direction = new Vector2(unitW.y, -unitW.x);
                line.point = rightCutoff.plus(unitW.scale(this.radius * invTimeHorizonObst));
                orcaLines.push(line);
                continue;
            }

            /* 
             * Project on left leg, right leg, or cut-off line, whichever is closest
             * to velocity.
             */
            distSqCutoff = ((t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? Infinity : RVOMath.absSq(this.velocity.minus(cutoffVec.scale(t).plus(leftCutoff))));
            distSqLeft = ((tLeft < 0.0) ? Infinity : RVOMath.absSq(this.velocity.minus(leftLegDirection.scale(tLeft).plus(leftCutoff))));
            distSqRight = ((tRight < 0.0) ? Infinity : RVOMath.absSq(this.velocity.minus(rightLegDirection.scale(tRight).plus(rightCutoff))));

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
            {
                /* Project on cut-off line. */
                line.direction = obstacle1.unitDir.scale(-1);
                var aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff); 
                orcaLines.push(line);
                continue;
            }
            else if (distSqLeft <= distSqRight)
            {
                /* Project on left leg. */
                if (isLeftLegForeign)
                {
                    continue;
                }

                line.direction = leftLegDirection;
                var aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff);
                orcaLines.push(line);
                continue;
            }
            else
            {
                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = rightLegDirection.scale(-1);
                var aux = new Vector2(-line.direction.y, line.direction.x);
                line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff);
                orcaLines.push(line);
                continue;
            }
        }

        var numObstLines = orcaLines.length;

        var invTimeHorizon = 1.0 / this.timeHorizon;

        /* Create agent ORCA lines. */
        for (var i = 0; i < this.agentNeighbors.length; ++i)
        {
            var other = this.agentNeighbors[i].value;

            relativePosition = other.position.minus(this.position);
            relativeVelocity = this.velocity.minus(other.velocity);
            distSq = RVOMath.absSq(relativePosition);
            combinedRadius = this.radius + other.radius;
            combinedRadiusSq = RVOMath.sqr(combinedRadius);

            var line = new Line(); // Line
            var u; // Vector2

            if (distSq > combinedRadiusSq)
            {
                /* No collision. */
                w = relativeVelocity.minus(relativePosition.scale(invTimeHorizon));; // Vector
                /* Vector from cutoff center to relative velocity. */
                wLengthSq = RVOMath.absSq(w);

                dotProduct1 = w.multiply(relativePosition);

                if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                {
                    /* Project on cut-off circle. */
                    var wLength = Math.sqrt(wLengthSq);
                    var unitW = w.scale(1 / wLength);

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    u = unitW.scale(combinedRadius * invTimeHorizon - wLength);
                }
                else
                {
                    /* Project on legs. */
                    leg = Math.sqrt(distSq - combinedRadiusSq);

                    if (RVOMath.det(relativePosition, w) > 0.0)
                    {
                        /* Project on left leg. */
                    	var aux = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg);
                        line.direction = aux.scale(1 / distSq);
                    }
                    else
                    {
                        /* Project on right leg. */
                    	var aux = new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg);
                        line.direction = aux.scale(-1 / distSq);
                    }

                    dotProduct2 = relativeVelocity.multiply(line.direction);

                    u = line.direction.scale(dotProduct2).minus(relativeVelocity);
                }
            }
            else
            {
                /* Collision. Project on cut-off circle of time timeStep. */
                invTimeStep = 1.0 / Simulator.instance.timeStep;

                /* Vector from cutoff center to relative velocity. */
                w = relativeVelocity.minus(relativePosition.scale(invTimeStep));

                wLength = Math.abs(w);
                unitW = w.scale(1 / wLength);

                line.direction = new Vector2(unitW.y, -unitW.x);
                u = unitW.scale(combinedRadius * invTimeStep - wLength);
            }

            line.point = u.scale(0.5).plus(this.velocity);
            orcaLines.push(line);
        }

        lineFail = this.linearProgram2(orcaLines, this.maxSpeed, this.prefVelocity, false);

        if (lineFail < orcaLines.length)
        {
            this.linearProgram3(orcaLines, numObstLines, lineFail, this.maxSpeed);
        }
    }

    this.insertAgentNeighbor = function(agent, rangeSq)
    {
        if (this != agent)
        {
            var distSq = RVOMath.absSq(this.position.minus(agent.position));

            if (distSq < rangeSq)
            {
                if (this.agentNeighbors.length < this.maxNeighbors)
                {
                    this.agentNeighbors.push(new KeyValuePair(distSq, agent));
                }
                var i = this.agentNeighbors.length - 1;
                while (i != 0 && distSq < this.agentNeighbors[i - 1].key)
                {
                    this.agentNeighbors[i] = this.agentNeighbors[i - 1];
                    --i;
                }
                this.agentNeighbors[i] = new KeyValuePair(distSq, agent);

                if (this.agentNeighbors.length == this.maxNeighbors)
                {
                    rangeSq = this.agentNeighbors[this.agentNeighbors.length-1].key;
                }
            }
        }
    }

    this.insertObstacleNeighbor = function(/** Obstacle */ obstacle, rangeSq)
    {
        nextObstacle = obstacle.nextObstacle;

        distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, this.position);

        if (distSq < rangeSq)
        {
            this.obstaclNeighbors.push(new KeyValuePair(distSq, obstacle));

            i = this.obstaclNeighbors.length - 1;
            while (i != 0 && distSq < this.obstaclNeighbors[i - 1].key)
            {
                this.obstaclNeighbors[i] = this.obstaclNeighbors[i - 1];
                --i;
            }
            this.obstaclNeighbors[i] = new KeyValuePair(distSq, obstacle);
        }
    }

    this.update = function()
    {
    	var rnd = new Vector2(0,0); //Math.random() * 0.1 - 0.05, Math.random() * 0.1 - 0.05);
        this.velocity = this.newVelocity.plus(rnd);
        this.position = this.position.plus(this.newVelocity.scale(Simulator.instance.timeStep));
    };

    this.linearProgram1 = function(/** Array */ lines, /** int */ lineNo, /** float */ radius, /** Vector2 */ optVelocity, /** bool */ directionOpt)
    {
        var dotProduct = lines[lineNo].point.multiply(lines[lineNo].direction);
        var discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

        if (discriminant < 0.0)
        {
            /* Max speed circle fully invalidates line lineNo. */
            return false;
        }

        var sqrtDiscriminant = Math.sqrt(discriminant);
        var tLeft = -dotProduct - sqrtDiscriminant;
        var tRight = -dotProduct + sqrtDiscriminant;

        for (var i = 0; i < lineNo; ++i)
        {
            var denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
            var numerator = RVOMath.det(lines[i].direction, lines[lineNo].point.minus(lines[i].point));

            if (Math.abs(denominator) <= RVOMath.RVOEPSILON)
            {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0)
                {
                    return false;
                }
                else
                {
                    continue;
                }
            }

            var t = numerator / denominator;

            if (denominator >= 0.0)
            {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.min(tRight, t);
            }
            else
            {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.max(tLeft, t);
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        if (directionOpt)
        {
            if (optVelocity.multiply(lines[lineNo].direction) > 0.0)
            {
                // Take right extreme
            	this.newVelocity = lines[lineNo].direction.scale(tRight).plus(lines[lineNo].point);
            }
            else
            {
                // Take left extreme.
            	this.newVelocity = lines[lineNo].direction.scale(tLeft).plus(lines[lineNo].point);
            }
        }
        else
        {
            // Optimize closest point
            t = lines[lineNo].direction.multiply(optVelocity.minus(lines[lineNo].point));

            if (t < tLeft)
            {
            	this.newVelocity = lines[lineNo].direction.scale(tLeft).plus(lines[lineNo].point);
            }
            else if (t > tRight)
            {
            	this.newVelocity = lines[lineNo].direction.scale(tRight).plus(lines[lineNo].point);
            }
            else
            {
            	this.newVelocity = lines[lineNo].direction.scale(t).plus(lines[lineNo].point);
            }
        }
        
        // TODO ugly hack by palmerabollo
        if (isNaN(this.newVelocity.x) || isNaN(this.newVelocity.y)) {
        	return false;
        }
        
        return true;
    }
    
    this.linearProgram2 = function(/** Array */ lines, /** float */ radius, /** Vector2 */ optVelocity, /** bool */ directionOpt)
    {
        if (directionOpt)
        {
            /* 
             * Optimize direction. Note that the optimization velocity is of unit
             * length in this case.
             */
        	this.newVelocity = optVelocity.scale(radius);
        }
        else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
        {
            /* Optimize closest point and outside circle. */
        	this.newVelocity = RVOMath.normalize(optVelocity).scale(radius);
        }
        else
        {
            /* Optimize closest point and inside circle. */
        	this.newVelocity = optVelocity;
        }

        for (var i = 0; i < lines.length; ++i)
        {
            if (RVOMath.det(lines[i].direction, lines[i].point.minus(this.newVelocity)) > 0.0)
            {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                var tempResult = this.newVelocity;
                if (!this.linearProgram1(lines, i, this.radius, optVelocity, directionOpt))
                {
                    this.newVelocity = tempResult;
                    return i;
                }
            }
        }

        return lines.length;
    }

    this.linearProgram3 = function(/** Array */ lines, /** int */ numObstLines, /** int */ beginLine, /** float */ radius)
    {
        var distance = 0.0;

        for (var i = beginLine; i < lines.length; ++i)
        {
            if (RVOMath.det(lines[i].direction, lines[i].point.minus(this.newVelocity)) > distance)
            {
                /* Result does not satisfy constraint of line i. */
                //std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);
                projLines = []; // new List<Line>();
                for (var ii = 0; ii < numObstLines; ++ii)
                {
                    projLines.push(lines[ii]);
                }

                for (var j = numObstLines; j < i; ++j)
                {
                    var line = new Line();

                    determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                    if (Math.abs(determinant) <= RVOMath.RVOEPSILON)
                    {
                        /* Line i and line j are parallel. */
                        if (lines[i].direction.multiply(lines[j].direction) > 0.0)
                        {
                            /* Line i and line j point in the same direction. */
                            continue;
                        }
                        else
                        {
                            /* Line i and line j point in opposite direction. */
                            line.point =lines[i].point.plus(lines[j].point).scale(0.5);
                        }
                    }
                    else
                    {
                    	var aux = lines[i].direction.scale(RVOMath.det(lines[j].direction, lines[i].point.minus(lines[j].point)) / determinant); 
                        line.point = lines[i].point.plus(aux);
                    }

                    line.direction = RVOMath.normalize(lines[j].direction.minus(lines[i].direction));
                    projLines.push(line);
                }

                var tempResult = this.newVelocity;
                if (this.linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y, lines[i].direction.x), true) < projLines.length)
                {
                    /* This should in principle not happen.  The result is by definition
                     * already in the feasible region of this linear program. If it fails,
                     * it is due to small floating point error, and the current result is
                     * kept.
                     */
                    this.newVelocity = tempResult;
                }

                distance = RVOMath.det(lines[i].direction, lines[i].point.minus(this.newVelocity));
            }
        }
    }
}