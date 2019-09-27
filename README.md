# Rigidbody2D

A simple 2D physics simulation with rigidbodies.

## Pipeline

```
step()
{
    List<Manifold> 'Ms'

    (optional block)
    for every rigidbody 'A'
    {
        Update its velocity by integration
    }
    (end)

    for every rigidbody 'A' from index [0, n)
    {
        for every "other" rigidbody 'B' from index [i+1, n)
        {
            Manifold 'M'
            if ( 'A' collides with 'B' )
            {
                'M' = info about the collision between 'A' and 'B'
                add 'M' to 'Ms'
            }
        }
    }

    for every Manifold 'M' in 'Ms'
    {
        Resolve collision 'M'
        // this is done by applying impulse, a change to velocity
    }

    for every rigidbody 'A'
    {
        Update its position and velocity by integration
    }

}

main loop()
{
    Accumulator 'Acc' = 0

    while(true)
    {
        Time 'curT' = currentTime()

        'Acc' += 'curT' - previousTime
        previousTime = 'curT'

        // clamp the accumulator to avoid overflowing, making system
        // go through too many steps in a single frame
        'Acc' = clamp( 'Acc', 0.0, 1.0 )

        while('Acc' > deltaTime)
        {
            step() // this refers to the function mentioned above
            'Acc' -= deltaTime
        }
    }
}

```

## Architecture

```
Scene
{
    List<Body> bodies;
}

Body (or Rigidbody2D)
{
    Shape s;
    Transform t;
    Material m;

    vec2 velocity;
    vec2 force;
    float mass;

    float staticFriction;
    float dynamicFriction;
    float restitution;
}

Manifold (or Collision2D)
{
    Body a, b; // the two objects that collide with each other
    float depth_of_penetration;
    vec2 normal;
    vec2 contacts[2]; // points of contact during collision
    uint contact_count; // number of contacts that occured during collision

    // some additional data for resolving collision
}

Shape (or Mesh2D)
{
    Body b; // referencing the body that uses this shape instance

    // some additional data for determining the 
    // actual shape of each shape instance
}

// implement classes that inherits 'Shape', 
// like a 'Circle', 'AABBRect', or 'Polygon'
```

# Reference

[Randy Gaul's Tutorial](https://gamedevelopment.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715)