function Vector2(x, y) {
    this.x = x;
    this.y = y;

    this.plus = function(vector) {
    	return new Vector2(x + vector.x, y + vector.y);
    };
    
    this.minus = function(vector) {
    	return new Vector2(x - vector.x, y - vector.y);
    };
    
    this.multiply = function(vector) {
    	return x * vector.x + y * vector.y;
    };
    
    this.scale = function(k) {
        return new Vector2(x*k, y*k);
    };
}

function Line() {
	/*
	this.point; // Vector2
	this.direction; // Vector2
	*/
}

function Obstacle() {
	/*
	this.point; // Vector2
	this.unitDir; // Vector2;
	this.isConvex; // boolean
	this.id; // int
	this.prevObstacle; // Obstacle
	this.nextObstacle; // Obstacle
	*/
}

function KeyValuePair(key, value) {
	this.key = key;
	this.value = value;
}

RVOMath = {};

RVOMath.RVO_EPSILON = 0.01;

RVOMath.absSq = function(v) {
    return v.multiply(v);
};

RVOMath.normalize = function(v) {
	return v.scale(1 / RVOMath.abs(v)); // v / abs(v)
};

RVOMath.distSqPointLineSegment = function(a, b, c) {
	var aux1 = c.minus(a);
	var aux2 = b.minus(a);
	
	// r = ((c - a) * (b - a)) / absSq(b - a);
	var r = aux1.multiply(aux2) / RVOMath.absSq(aux2);
	
	if (r < 0) {
		return RVOMath.absSq(aux1); // absSq(c - a)
	} else if (r > 1) {
		return RVOMath.absSq(aux2); // absSq(c - b)
	} else {
		return RVOMath.absSq( c.minus(a.plus(aux2.scale(r))) ); // absSq(c - (a + r * (b - a)));
	}
};

RVOMath.sqr = function(p) {
	return p * p;
};

RVOMath.det = function(v1, v2) {
	return v1.x * v2.y - v1.y* v2.x;
};

RVOMath.abs = function(v) {
	return Math.sqrt(RVOMath.absSq(v));
};

RVOMath.leftOf = function(a, b, c) {
	return RVOMath.det(a.minus(c), b.minus(a));
};