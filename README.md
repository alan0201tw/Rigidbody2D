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

    for every rigidbody 'A'
    {
        for every "other" rigidbody 'B'
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
```

## Architecture



# Reference

[Randy Gaul's Tutorial](https://gamedevelopment.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715)