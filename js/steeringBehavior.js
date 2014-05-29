// **code originally from:** [http://rocketmandevelopment.com/blog](rocketmandevelopment)
// **author:** Michael Trouw
// **web:** http://www.powergeek.nl/
//
//
//
// The Flee behavior makes a boid flee from another boid, or object, or anything that has
// a getVector() method that returns a Vector2D.
//
// The Flee constructor takes 2 arguments:
//
// + *the target to flee from*
// + *the minimal distance to the target*
// The difference in pixels between the object to seek and the boid that has this behavior.
// if not, the seek difference will be set to 25 by default.



// The Follow behavior makes a boid follow another boid, or object, or anything that has
// a getVector() method that returns a Vector2D, usually a Vector2D object.
//
// The follow constructor takes 2 arguments:
//
// + *the target to follow*
// + *the minimal distance to the target*
// The difference in pixels between the object to follow and the boid that has this behavior.
// if not, the seek difference will be set to 25 by default.


function Follow( target, minDistance ) {

    this.minDistance = minDistance || 25;
    this.target = target;
}
Follow.prototype = {
    force: function( velocity, position, maxSpeed, mass ) {

        var targetPosition = this.target.getVector();
        var distance = targetPosition.cloneVector().subtract(position);

        if ( Math.abs( distance.x ) < this.minDistance && Math.abs(distance.y) < this.minDistance) {
        return new Vector2D(0,0);
        } else {
        var desiredVelocity = targetPosition.cloneVector().subtract(position).normalize().multiply(maxSpeed);
        var steeringForce = desiredVelocity.subtract(velocity);
        var outputForce = steeringForce.divide(mass);

        return outputForce;
        }
    },
    toString: function() {
        return "Follow";
    },
    getFollowTarget: function() {
        return this.target;
    }
}

function Arrive( target, minDistance, timeToTarget ) {
    this.target = target;
    this.minDistance = minDistance || 25;
    this.timeToTarget = timeToTarget || 3;
}

Arrive.prototype = {
    force: function( velocity, position, maxSpeed, mass ) {

        var targetPosition = this.target.getVector();
        var distance = targetPosition.cloneVector().subtract( position );

        if ( distance.length() < this.minDistance ) {
        return new Vector2D(0,0);
        } else {
            var desiredVelocity = targetPosition.cloneVector().subtract( position );//.normalize().multiply(maxSpeed);
            var steeringForce = desiredVelocity.multiply( 1 / this.timeToTarget );

            if ( steeringForce.length() > this.target.maxSpeed() * this.target.maxSpeed() ) {
                steeringForce.unit().multiply(this.target.maxSpeed());
            }

            var outputForce = steeringForce.divide( mass );
            return outputForce;
        }
    },
    toString: function() {
        return "Flee";
    },
    getFleeTarget: function() {
        return this.target;
    }  
}

// **code originally from:** [http://rocketmandevelopment.com/blog](rocketmandevelopment)
// **author:** Michael Trouw
// **web:** http://www.powergeek.nl/
//
//
//
// The Flee behavior makes a boid flee from another boid, or object, or anything that has
// a getVector() method that returns a Vector2D.
//
// The Flee constructor takes 2 arguments:
//
// + *the target to flee from*
// + *the minimal distance to the target*
// The difference in pixels between the object to seek and the boid that has this behavior.
// if not, the seek difference will be set to 25 by default.

function Arrive2( target, minDistance, timeToTarget ) {
    this.target = target;
    this.minDistance = minDistance || 25;
    this.timeToTarget = timeToTarget || 3;
    this.maxSpeed = 5;
    this.radius = 5;
}
Arrive2.prototype = {
    force: function( velocity, position, maxSpeed, mass ) {

        var targetPosition = this.target.getVector();
        var distance = targetPosition.cloneVector().subtract( position );

        if ( distance.length() < this.minDistance ) {
            return new Vector2D(0, 0);
        } else {
            var desiredVelocity = targetPosition.cloneVector().subtract( position );//.normalize().multiply(maxSpeed);
            var steeringForce;

            if (desiredVelocity.length() < (this.radius * this.radius)) {
            return new Vector2D(0, 0);
            } else {
                steeringForce = desiredVelocity.multiply( 1 / this.timeToTarget );

                if (steeringForce.length() > this.maxSpeed * this.maxSpeed) {
                    steeringForce.unit().multiply( this.maxSpeed );
                }
            }
            var outputForce = steeringForce.divide( mass );
            return outputForce;
        }
    },
    toString: function() {
        return "Flee";
    },
    getFleeTarget: function() {
        return this.target;
    }
}

function BlendedSteering( weightedbehaviors ) {
    if(weightedbehaviors != null || weightedbehaviors != undefined){
    this.behaviors = weightedbehaviors
    } else {
    this.behaviors = undefined;
    }

    this.combinedForce = new Vector2D();
}
BlendedSteering.prototype = {
    force: function( boidVelocity ) {

        var combinedForce = this.combinedForce;
    
        for ( var i = 0; i < this.behaviors.length; i++ ) {
            var behavior = this.behaviors[i];
            var behaviorForce = behavior.force( boidVelocity );
            combinedForce.add( behaviorForce.multiply( behavior.weight ) );
        }
    
        return combinedForce;
    },
    add: function(behavior){
        if ( this.behaviors == undefined ){
        this.behaviors = new Array();
        }
    
        this.behaviors.push( behavior );
    },
    getAll: function(){
        return this.behaviors;
    }
}

// **code originally from:** [http://rocketmandevelopment.com/blog](rocketmandevelopment)
// **author:** Michael Trouw
// **web:** http://www.powergeek.nl/
//
//
//
// The Flee behavior makes a boid flee from another boid, or object, or anything that has
// a getVector() method that returns a Vector2D.
//
// The Flee constructor takes 2 arguments:
//
// + *the target to flee from*
// + *the minimal distance to the target*
// The difference in pixels between the object to seek and the boid that has this behavior.
// if not, the seek difference will be set to 25 by default.

function CircleTarget( target, distance, offset ) {

    this.distance = distance;
    this.offset = offset;
    this.target = target;
}
CircleTarget.prototype = {
    force: function( velocity, position, maxSpeed, mass ) {

        var targetPosition = this.target.getVector();
    
        // if(this.distance > 0 && position.distSQ(targetPosition) > (this.distance * this.distance)) {
        // return velocity;
        // }
    
        var normalized = position.copyAndSubtract( targetPosition ).unit();
        var newTarget = targetPosition.copyAndAdd( normalized.multiply( this.offset ) );
        
        //debug draw the newTarget position 
    
        newTarget.subtract(position);
        return newTarget.add(velocity).divide(mass);
    },
    toString: function() {
        return "CircleTarget";
    },
    getCircleTarget: function() {
        return this.target;
    }
}

/*
Description: balances behaviors.
If there's 2 behaviors, each will have a 50% weight.
3 behaviors, each will have 33% weight etc. etc.
*/

function Weighing( behaviors ) {

    if ( behaviors != null || behaviors != undefined ){
        this.behaviors = behaviors;
        } else {
        this.behaviors = undefined;
        }
}
Weighing.prototype = {
    balance: function( behaviors ) {

        for ( var i = 0; i < behaviors.length; i++ ) {
        var behavior = behaviors[i];
        behavior.weight = (1 / behaviors.length);
        console.log("behavior weight = " + behavior.weight);
        }
    
        console.log("behaviors have been balanced.");
        return behaviors;
    },

    prioritize: function() {
    // TODO: prioritize behavior weights
    },

    randomize: function() {
    // TODO: randomize behavior weights for fun :)
    }
}

function Weightedbehavior( behavior, weight ) {

    this.behavior = behavior;
    this.weight = weight;
}
Weightedbehavior.prototype = {

    force: function( velocity ) {
        return this.behavior.force( velocity );
    }
}

function Flee( target, minDistance ) {

    this.minDistance = minDistance || 25;
    this.target = target;
}
Flee.prototype = {
    force: function( velocity, position, maxSpeed, maxAcceleration, mass ) {

        var toTarget = this.target.position().cloneVector().subtract( position );

        if ( Math.abs( toTarget.x ) < this.minDistance && Math.abs( toTarget.y ) < this.minDistance ) {
            return new Vector2D( 0,0 );
        } else {
            var desiredVelocity = toTarget.normalize().multiply( maxSpeed );
            var steeringForce = desiredVelocity.subtract( velocity );
            steeringForce = steeringForce.truncate( maxAcceleration );
            steeringForce = steeringForce.divide( mass ).multiply( -1 );
            return steeringForce;
        }
    },
    toString: function() {
        return "Flee";
    },
    getFleeTarget: function() {
        return this.target;
    }
}

function Evade( target, minDistance, lookAheadScale ) {

    this.minDistance = minDistance || 25;
    this.target = target;
    this.lookAheadScale = lookAheadScale || 0.1;
}
Evade.prototype = {
    force: function( velocity, position, maxSpeed, maxAcceleration, mass ) {

        var distanceToTarget = this.target.position().distance( position );
        var lookAheadTime = distanceToTarget * this.lookAheadScale;

        var targetPosition = this.target.position().cloneVector();
        var targetVelocity = this.target.velocity().cloneVector();
        var futureTarget = targetPosition.add( targetVelocity.multiply ( lookAheadTime ) );

        var targetBoid = utils.object( this.target );
        targetBoid.position( futureTarget ); 
        var flee = new Flee( targetBoid, this.minDistance );

        return flee.force( velocity, position, maxSpeed, maxAcceleration, mass );
    },
    toString: function() {
        return "Evade";
    },
    getEvadeTarget: function() {
        return this.target;
    }
}
// **code originally from:** [http://rocketmandevelopment.com/blog](rocketmandevelopment)
// **author:** Michael Trouw
// **web:** http://www.powergeek.nl/
//
//
//
// The Seek behavior makes a boid seek another boid, or object, or anything that has
// a getVector() method that returns a Vector2D.
//
// The seek constructor takes 2 arguments:
//
// + *the target to seek*
// + *the minimal distance to the target*
// The difference in pixels between the object to seek and the boid that has this behavior.
// if not, the seek difference will be set to 25 by default.

function Seek( target, minDistance ) {

    this.minDistance = minDistance || 25;
    this.target = target;
}


Seek.prototype = {
    force: function( velocity, position, maxSpeed, maxAcceleration, mass ) {

        var toTarget = this.target.position().cloneVector().subtract( position );

        if ( Math.abs( toTarget.x ) < this.minDistance && Math.abs( toTarget.y ) < this.minDistance ) {
            return new Vector2D( 0,0 );
        } else {
            var desiredVelocity = toTarget.normalize().multiply( maxSpeed );
            var steeringForce = desiredVelocity.subtract( velocity );
            steeringForce = steeringForce.truncate( maxAcceleration );
            steeringForce = steeringForce.divide( mass );
            return steeringForce;
        }
    },
    toString: function() {
        return "Seek";
    },
    getSeekTarget: function() {
        return this.target;
    }
}

function Pursue( target, minDistance, lookAheadScale ) {

    this.minDistance = minDistance || 25;
    this.target = target;
    this.lookAheadScale = lookAheadScale || 0.3;
}

/*
var distance:Number = target.position.distance(position);
var T:Number = distance / target._maxSpeed;
var targetPosition:Vector2D = target.position.cloneVector().add(target.velocity.cloneVector().multiply(T));
seek(targetPosition);
*/

Pursue.prototype = {
    force: function( velocity, position, maxSpeed, maxAcceleration, mass ) {

        var distanceToTarget = this.target.position().distance( position );
        var lookAheadTime = distanceToTarget * this.lookAheadScale;

        var targetPosition = this.target.position().cloneVector();
        var targetVelocity = this.target.velocity().cloneVector();
        var futureTarget = targetPosition.add( targetVelocity.multiply ( lookAheadTime ) );

        var targetBoid = utils.object( this.target );
        targetBoid.position( futureTarget ); 
        var seek = new Seek( targetBoid, this.minDistance );

        return seek.force( velocity, position, maxSpeed, maxAcceleration, mass );
    },
    toString: function() {
        return "Pursue";
    },
    getPursueTarget: function() {
        return this.target;
    }
}
// **code originally from:** [http://rocketmandevelopment.com/blog](rocketmandevelopment)
// **author:** Michael Trouw
// **web:** http://www.powergeek.nl/
//
//
//
// The Wander behavior makes a boid wander around, which means not moving in a particular direction.
//
// The Wander constructor can take three arguments:
//

function Wander( properties ) {
    this.circleRadius = properties.circleRadius || 10;
    this.wanderAngle = properties.wanderAngle || 5;
    this.wanderChange = properties.wanderChange || 1;

    this.circleRadius = 5 + Math.random() * this.circleRadius;
    this.wanderAngle *= Math.random();
    this.wanderChange *= Math.random();
}
Wander.prototype = {
    force: function( velocity ) {
        var circleMiddle = velocity.cloneVector().normalize().multiply( this.circleRadius );
        var wanderForce = new Vector2D();
        wanderForce.length( 0.2 );
        wanderForce.angle( this.wanderAngle );
        this.wanderAngle += Math.random() * this.wanderChange - this.wanderChange * 0.5;
        var outputForce = circleMiddle.add( wanderForce );
        return outputForce;
    }
}

function Boid( properties ) {
   
    // basic properties for boid (can be passed in constructor object)
    this._mass = properties.mass || 20;
    this._maxSpeed = properties.maxSpeed || 4;
    this._maxAcceleration = properties.maxAcceleration || 3;
    this.behaviors = properties.behaviors || undefined;	// behavior data
    //this.pathIndex = properties.pathIndex;

    // other vector properties
    this._position = new Vector2D( properties.x, properties.y );
    this._velocity = new Vector2D();
    this.rotation = 0;

    // behavior specifics
    this.seekObject = undefined;
    this.followObject = undefined;
    this.evadeObject = undefined;

    this.radius = properties.radius || 20;
    this.gunWidth = properties.gunWidth || 14;
    this.gunHeight = properties.gunHeight || 27;

    // drawing graphics
    this.graphicsForDrawing = new Graphics( properties.graphicsForDrawing ); 
}
Boid.prototype = {
    toString: function() {
        return Math.random() * 12345678;
    },

    /**
    * Updates the boid based on velocity
    */
    update: function() {

        this._velocity.truncate( this._maxSpeed );    // keep it witin its max speed
        this._position = this._position.add( this._velocity );    // move it
        this.rotation = this._velocity.angle() * 180 / Math.PI; // rotation = the velocity's angle converted to degrees
        this._velocity.add( this.behaviors.force( this._velocity, this._position, this._maxSpeed, this._maxAcceleration, this._mass ) );  // execute behavior 

        //clamp boid to screen
        if ( this.x < -this.radius ) {
            this.x = canvas.width + this.radius;
        } else if ( this.x > canvas.width + this.radius ) {
            this.x = -this.radius;
        }
        
        if ( this.y < -this.radius ) {
            this.y = canvas.height + this.radius;
        } else if (this.y > canvas.height + this.radius) {
            this.y = 0;
        }
    },
    draw: function() {  //param1:Graphics, param2: uint

        this.graphicsForDrawing.save();
        this.graphicsForDrawing.translate ( this._position.x, this._position.y );
        this.graphicsForDrawing.rotate( this._velocity.angle() );

        //draw circular gun base
        this.graphicsForDrawing.beginPath();
        //x, y, radius, start_angle, end_angle, anti-clockwise
        this.graphicsForDrawing.circle( 0, 0, this.radius);
        this.graphicsForDrawing.closePath();
        this.graphicsForDrawing.fill();
        this.graphicsForDrawing.stroke();

        //draw rectangular gun

        this.graphicsForDrawing.translate( 0, - this.gunWidth / 2 );
        this.graphicsForDrawing.fillRect( 0, 0, this.gunHeight, this.gunWidth);
        this.graphicsForDrawing.strokeRect( 0, 0, this.gunHeight, this.gunWidth);
        this.graphicsForDrawing.restore();


        //this.vectorGraphics.save();
        //this.vectorGraphics.translate( this._position.x, this._position.y )
        //this._velocity.draw( this.vectorGraphics );
        //this.behaviors.force.draw( this.vectorGraphics, "#FF0000");
        //this.vectorGraphics.restore();
    },
    /**
    * checks if boid is in range of some other vector
    */
    isWithinRange: function( vector, range ) {
        if ( this._position.distance( vector ) < range )
            return true;
        else
            return false;
    },
    /**
    * Gets and sets the boid's mass - GETTER AND SETTER!
    */
    mass: function( value ) {
        if ( value ) {
            this._mass = value;
        } else {
            return this._mass;
        }   
    },

    /**
    * Gets and sets the max speed of the boid - GETTER AND SETTER!
    */
    maxSpeed: function( value ) {
        if ( value ) {
            this._maxSpeed = value;
        } else {
            return this._maxSpeed;
        }
    },

    /**
    *Gets and sets the position of the boid - GETTER AND SETTER!
    */
    position: function( value ) {
        if ( value ) {
            this._position = value;
        // this.x(this._position.x);
        // this.y(this._position.y);
        } else {
            return this._position;
        }
    },
    /**
    * Gets and sets the velocity of the boid - GETTER AND SETTER!
    */
    velocity: function( value ) {
        if ( value ) {
            this._velocity = value;
        } else {
            return this._velocity;
        }
    },
    getVector: function() {
        return this.position();
    },
    getPathIndex: function() {
        return this.pathIndex;
    }
}
/**
*Gets and sets the X position of the boid - GETTER AND SETTER!
*/
Object.defineProperty (Boid.prototype, "x", {
        get: function () { return this._position.x; },
        set: function ( new_value ) {
               this._position.x = new_value;
             }
});
/**
*Gets and sets the Y position of the boid - GETTER AND SETTER!
*/
Object.defineProperty (Boid.prototype, "y", {
        get: function () { return this._position.y; },
        set: function ( new_value ) {
               this._position.y = new_value;
             }
});

/*
Original Author: com.rocketmandevelopment.math (resolve this)
Ported by: Michael Trouw
*/

/**
* Vector2D Constructor
*/
function Vector2D( x, y ) {
   
    this.self = this; // :Vector2D

    if ( x == undefined && y == undefined ){
        x = 0;
        y = 0;
    }
    this.x = x;
    this.y = y;
}

Vector2D.prototype = {
    /**
    * Creates an exact copy of this Vector2D
    * @return Vector2D A copy of this Vector2D
    */
    cloneVector: function() { // returns: Vector2D
        return new Vector2D( this.x, this.y );
    },
    /**
    * Makes x and y zero.
    * @return Vector2D This vector.
    */
    zeroVector: function() { // returns: Vector2D
        this.x = 0;
        this.y = 0;
        return this.self;
    },
    /**
    * Is this vector zeroed?
    * @return Boolean Returns true if zeroed, else returns false.
    */
    isZero: function() { // returns: Boolean
        return this.x == 0 && this.y == 0;
    },
    /**
    * Is the vector's length = 1?
    * @return Boolean If length is 1, true, else false.
    */
    isNormalized: function() { // returns: Boolean
        return this.length == 1.0;
    },
    /**
    * Does this vector have the same location as another?
    * @param vector2 The vector to test.
    * @return Boolean True if equal, false if not.
    */
    equals: function( vector2d ) {    // param: Vector2D returns: Boolean
        return this.x == vector2d.x() && this.y == vector2d.y();
    },
    /**
    * Sets the length which will change x and y, but not the angle and gets the length - GETTER AND SETTER!
    */
    length: function( value ) {   // param :Number
        if ( value ) {
            var _angle = this.angle();  // :Number
            this.x = Math.cos(_angle) * value;
            this.y = Math.sin(_angle) * value;
            if(Math.abs(this.x) < 0.00000001) this.x = 0;
            if(Math.abs(this.y) < 0.00000001) this.y = 0;
        } else {
            return Math.sqrt( this.lengthSquared() );
        }
    },
    /**
    * Returns the length of this vector, before square root. Allows for a faster check.
    */
    lengthSquared: function() {   // returns: Number GETTER!
        return this.x * this.x + this.y * this.y;
    },
    /**
    * Changes the angle of the vector. X and Y will change, length stays the same. Get the angle of this vector.
    */
    angle: function( value ) { //param: Number SETTER!
        if ( value ) {
            var len = this.length();    // :Number
            this.x = Math.cos( value ) * len;
            this.y = Math.sin( value ) * len;
        } else {
            return Math.atan2( this.y, this.x );
        }
    },

    /**
    * Sets the vector's length to 1.
    * @return Vector2D This vector.
    */
    normalize: function() {   // returns :Vector2D
        if ( this.length() == 0 ){
            this.x = 1;
            return this.self;
        }
        var len = this.length();    // :Number
        this.x /= len;
        this.y /= len;
        return this.self;
    },
    /**
    * Sets the vector's length to len.
    * @param len The length to set it to.
    * @return Vector2D This vector.
    */
    normalcate: function( len ) {   // param :Number returns :Vector2D
        this.length( len );
        return this.self;
    },
    /**
    * Sets the length under the given value. Nothing is done if the vector is already shorter.
    * @param max The max length this vector can be.
    * @return Vector2D This vector.
    */
    truncate: function( max ) { // param: Number returns :Vector2D
        this.length( Math.min( max, this.length() ) );
        return this.self;
    },
    /**
    * Makes the vector face the opposite way.
    * @return Vector2D This vector.
    */
    reverse: function() { // returns: Vector2D
        this.x = -this.x;
        this.y = -this.y;
        return this.self;
    },
    /**
    * Calculate the dot product of this vector and another.
    * @param vector2 Another vector2D.
    * @return Number The dot product.
    */
    dotProduct: function( vector2 ) {   // param: Vector2D returns :Number
        return this.x * vector2.x + this.y * vector2.y;
    },
    /**
    * Calculate the cross product of this and another vector.
    * @param vector2 Another Vector2D.
    * @return Number The cross product.
    */
    crossProd: function( vector2 ) {    // param :Vector2D returns :Number
        return this.x * vector2.y - this.y * vector2.x;
    },
    /**
    * Calculate angle between any two vectors.
    * @param vector1 First vector2d.
    * @param vector2 Second vector2d.
    * @return Number Angle between vectors.
    */
    angleBetween: function( vector1, vector2 ) {    // param1 :Vector2D, param2 :Vector2D, returns :Number
        if( !vector1.isNormalized() ) vector1 = vector1.cloneVector().normalize();
        if( !vector2.isNormalized() ) vector2 = vector2.cloneVector().normalize();
        return Math.acos( vector1.dotProduct( vector2 ));
    },
    /**
    * Is the vector to the right or left of this one?
    * @param vector2 The vector to test.
    * @return Boolean If left, returns true, if right, false.
    */
    sign: function( vector2 ) { // param :Vector2D returns: int
        return perpendicular.dotProduct( vector2 ) < 0 ? -1 : 1;
    },
    /**
    * Get the vector that is perpendicular.
    * @return Vector2D The perpendicular vector.
    */
    perpendicular: function() {   // returns: Vector2D GETTER!
        return new Vector2D( -this.y, this.x) ;
    },
    /**
    * Calculate between two vectors.
    * @param vector2 The vector to find distance.
    * @return Number The distance.
    */
    distance: function( vector2 ) { //param :Vector2D returns: Number
        //console.log(vector2);
        return Math.sqrt(this.distSQ( vector2 ));
    },
    /**
    * Calculate squared distance between vectors. Faster than distance.
    * @param vector2 The other vector.
    * @return Number The squared distance between the vectors.
    */
    distSQ: function( vector2 ) {   // param :Vector2D returns :Number
        //console.log(vector2);
        var dx = vector2.x - this.x;    // :Number
        var dy = vector2.y - this.y;    // :Number
        return ( dx * dx ) + ( dy * dy );
    },
    /**
    * Add a vector to this vector.
    * @param vector2 The vector to add to this one.
    * @return Vector2D This vector.
    */
    add: function( vector2 ) {  //param: Vector2D, returns: Vector2D
        this.x += vector2.x;
        this.y += vector2.y;
        return this.self;
    },
    /**
    * Returns the value of the given vector added to this.
    */
    // this method was taken from Shane McCartney's Vector 2D implementation
    copyAndAdd: function( vector2 ) {
        return new Vector2D( this.x + vector2.x, this.y + vector2.y );
    },
    /**
    * Subtract a vector from this one.
    * @param vector2 The vector to subtract.
    * @return Vector2D This vector.
    */
    subtract: function( vector2 ) { //param: Vector2D, returns: Vector2D
        this.x -= vector2.x;
        this.y -= vector2.y;
        return this.self;
    },
    /**
    * Returns the value of the given vector subtracted from this.
    * in other words: copy and subtract
    */
    // this method was taken from Shane McCartney's Vector 2D implementation
    copyAndSubtract: function( vector2 ) {
        return new Vector2D( this.x - vector2.x, this.y - vector2.y );
    },
    /**
    * Mutiplies this vector by another one.
    * @param scalar The scalar to multiply by.
    * @return Vector2D This vector, multiplied.
    */
    multiply: function( scalar ) {  //param: Number returns: Vector2D
        this.x *= scalar;
        this.y *= scalar;
        return this.self;
    },
    /**
    * Divide this vector by a scalar.
    * @param scalar The scalar to divide by.
    * @return Vector2D This vector.
    */
    divide: function( scalar ) {    //param:Number returns :Vector2D
        this.x /= scalar;
        this.y /= scalar;
        return this.self;
    },
    /**
    * Create a vector2D from a string
    * @param string The string to turn into a vector. Must be in the toString format.
    * @return Vector2D The vector from the string.
    **/
    fromString: function( string ) { // param: String returns: Vector2D
        var vector = new Vector2D();    // :Vector2D
        var tx; //:Number
        var ty; //:Number
        tx = parseInt( string.substr( string.indexOf( "x:" ), string.indexOf( "," ) ), 10 );
        ty = parseInt( string.substr( string.indexOf( "y:" ) ), 10 );
        vector.x = tx;
        vector.y = ty;
        return vector;
    },
    /**
    * Turn this vector into a string.
    * @return String This vector in string form.
    */
    toString: function() {    //returns: String
        return ("Vector2D x:" + this.x + ", y:" + this.y);
    },

    /**
    * Draw vector, good to see where its pointing.
    * @param graphicsForDrawing The graphics to draw the vector.
    * @param drawingColor The color to draw the vector in.
    */
    draw: function( graphicsForDrawing, strokeColor, lineWidth ) {  //param1:Graphics, param2: uint

        graphicsForDrawing.save();
        graphicsForDrawing.lineStyle( strokeColor, lineWidth );
        graphicsForDrawing.beginPath();
        graphicsForDrawing.moveTo( 0, 0 );
        graphicsForDrawing.lineTo( this.x, this.y );
        graphicsForDrawing.restore();
    },
    /**
    * Turns a non-zero vector into a vector of unit length.
    */
    unit: function() {
        var l = this.magnitude();

        if ( l !== 0 ) {
            this.x *= 1 / l;
            this.y *= 1 / l;
        }

        return this.self;
    },
    /**
    * Gets the magnitude of this vector.
    */
    magnitude: function() {
        return Math.sqrt( this.x * this.x + this.y * this.y );
    }

}

/**
* Graphics Constructor
*/
function Graphics( properties ) {
    
    this.context = properties.context;
    this.strokeStyle = ( properties.strokeStyle == undefined ) ? "#000" : properties.strokeStyle;
    this.fillStyle = ( properties.fillStyle == undefined ) ? "#00FF00" : properties.fillStyle; 
    this.lineWidth = ( properties.lineWidth == undefined ) ? 2 : properties.lineWidth;
  
    this.self = this;
}

Graphics.prototype = {
    /**
    * Draws line to context
    * @param lineWidth The width of the line
    * @param drawingColor The color of the line
    */
    lineStyle: function( strokeColor, lineWidth ) { 
        this.context.strokeStyle = ( strokeStyle == undefined ) ? this.strokeStyle : strokeStyle;
        this.context.lineWidth = ( lineWidth == undefined ) ? this.lineWidth : lineWidth;
        if ( this.context.lineWidth > 0 ) {
            this.context.stroke();
        }  
        return this.self; 
    },
    fillRect: function( x, y, width, height, fillStyle ) {  
        this.context.fillStyle = ( fillStyle == undefined ) ? this.fillStyle : fillStyle;
        this.context.fillRect( x, y, width, height );
        return this.self; 
    },
    strokeRect: function( x, y, width, height, strokeStyle ) {  
        this.context.strokeStyle = ( strokeStyle == undefined ) ? this.strokeStyle : strokeStyle;
        this.context.lineWidth = this.lineWidth;
        this.context.strokeRect( x, y, width, height );
        return this.self; 
    },
    fill: function( fillStyle ) {  
        this.context.fillStyle = ( fillStyle == undefined ) ? this.fillStyle : fillStyle;
        this.context.fill();
        return this.self; 
    },    
    stroke: function( strokeStyle ) { 
        this.context.strokeStyle = ( strokeStyle == undefined ) ? this.strokeStyle : strokeStyle;
        this.context.lineWidth = this.lineWidth;
        this.context.stroke();
        return this.self; 
    },            
    translate: function( x, y ) {
        this.context.translate( x, y);
        return this.self;
    },
    rotate: function( angle ) {
        this.context.rotate( angle );
        return this.self;
    },    
    moveTo: function( x, y ) {
        this.context.moveTo( x, y);
        return this.self;
    },
    beginPath: function() {
        this.context.beginPath();
        return this.self;  
    },
    closePath: function() {
        this.context.closePath();
        return this.self;  
    },   
    lineTo: function( x, y ) {
        this.context.lineTo( x, y);
        return this.self;
    },
    circle: function( x, y, radius ) {
        this.context.arc( x, y, radius, 0, Math.PI * 2, false );
        return this.self;
    },    
    clear: function(){
        this.context.clearRect( this.x, this.y, this.width, this.height );
    },
    save: function(){
        this.context.save();
    },
    restore: function(){
        this.context.restore();
    }      

}